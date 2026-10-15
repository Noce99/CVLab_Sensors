//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Usage: save_point_cloud_livox <output_folder> [livox_config.json] [options]
//   --pattern N        scan pattern: 0 non-repetitive, 1 repetitive, 2 repetitive low frame rate
//   --dual-emit 0|1    dual emit (two returns)
//   --blind-spot CM    blind spot distance, cm
//   --report PATH      write the settings the lidar reports back, as JSON (after applying the options)
// Options left out keep what the lidar has (the livox_config.json lidar_configs are not sent by the SDK).
//
// Records the Livox point cloud into H5 files (see h5_point_writer.hpp).
// Stop with SIGINT/SIGTERM: the last partial chunk is flushed before exiting.

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "h5_point_writer.hpp"

static const size_t MAX_POINTS_PER_FILE = 100000;
static const char *DEFAULT_CONFIG = "/home/ubm/CVLab_Sensors/lidar_recorders/livox_hap.json";

static std::unique_ptr<H5PointWriter> writer;

static std::atomic<uint64_t> points_received{0};
static std::atomic<int> last_time_type{-1};
static std::atomic<bool> warned_unsupported{false};

// Settings sent when the lidar connects; -1 = keep what the lidar has
struct Settings
{
  int pattern = -1;
  int dual_emit = -1;
  long blind_spot_cm = -1;
  std::string report_path;
};
static Settings settings;
static std::mutex notes_mutex;
static std::vector<std::string> notes;   // settings the lidar refused
static std::atomic<int> pending_sets{0};

// Livox time_type: 0 = no sync (time since power on), 1 = PTP/gPTP, 2 = GPS
static const char *time_type_name(int time_type)
{
  switch (time_type)
  {
  case 0:
    return "livox_unsynced";
  case 1:
    return "livox_ptp";
  case 2:
    return "livox_gps";
  default:
    return "livox_unknown";
  }
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data)
{
  if (data == nullptr)
    return;

  const int64_t host_ns = host_now_ns();

  uint64_t packet_ts = 0;
  for (int i = 0; i < 8; i++)
    packet_ts |= static_cast<uint64_t>(data->timestamp[i]) << (8 * i);
  const int64_t first_point_ns = static_cast<int64_t>(packet_ts);

  if (data->time_type != last_time_type)
  {
    last_time_type = data->time_type;
    writer->set_time_source(time_type_name(data->time_type));
    std::printf("Livox time source: %s\n", time_type_name(data->time_type));
    std::fflush(stdout);
  }
  writer->add_sync(first_point_ns, host_ns);

  // time_interval: whole packet duration, unit 0.1 us
  const int64_t packet_duration_ns = static_cast<int64_t>(data->time_interval) * 100;
  const uint32_t dot_num = data->dot_num;

  if (data->data_type == kLivoxLidarCartesianCoordinateHighData)
  {
    auto *p = reinterpret_cast<LivoxLidarCartesianHighRawPoint *>(data->data);
    for (uint32_t i = 0; i < dot_num; i++)
    {
      if (p[i].x == 0 && p[i].y == 0 && p[i].z == 0)
        continue;
      writer->add_point(static_cast<float>(p[i].x), static_cast<float>(p[i].y), static_cast<float>(p[i].z),
                        first_point_ns + packet_duration_ns * i / dot_num, p[i].reflectivity);
    }
  }
  else if (data->data_type == kLivoxLidarCartesianCoordinateLowData)
  {
    auto *p = reinterpret_cast<LivoxLidarCartesianLowRawPoint *>(data->data);
    for (uint32_t i = 0; i < dot_num; i++)
    {
      if (p[i].x == 0 && p[i].y == 0 && p[i].z == 0)
        continue;
      // Low resolution data is in cm
      writer->add_point(p[i].x * 10.0f, p[i].y * 10.0f, p[i].z * 10.0f,
                        first_point_ns + packet_duration_ns * i / dot_num, p[i].reflectivity);
    }
  }
  else if (!warned_unsupported.exchange(true))
  {
    std::fprintf(stderr, "WARNING: unsupported Livox data_type %u (set pcl_data_type 1 in the config)\n", data->data_type);
  }

  points_received += dot_num;
}

void WorkModeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data)
{
}

static std::string json_escape(const std::string &text)
{
  std::string out;
  for (char c : text)
  {
    if (c == '"' || c == '\\')
      out += '\\';
    out += c;
  }
  return out;
}

// Write the settings the lidar reports, atomically (the server may read it at any time)
static void write_report(const std::string &readback_json)
{
  std::string notes_json;
  {
    std::lock_guard<std::mutex> lock(notes_mutex);
    for (size_t i = 0; i < notes.size(); i++)
      notes_json += (i ? ", \"" : "\"") + json_escape(notes[i]) + "\"";
  }
  const std::string json = "{\"readback\": " + readback_json + ", \"notes\": [" + notes_json + "]}";
  std::printf("Livox settings: %s\n", json.c_str());
  std::fflush(stdout);
  if (settings.report_path.empty())
    return;
  const std::string tmp = settings.report_path + ".tmp";
  FILE *f = std::fopen(tmp.c_str(), "w");
  if (f == nullptr)
  {
    std::fprintf(stderr, "WARNING: cannot write %s\n", tmp.c_str());
    return;
  }
  std::fputs(json.c_str(), f);
  std::fclose(f);
  std::rename(tmp.c_str(), settings.report_path.c_str());
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle,
                               LivoxLidarDiagInternalInfoResponse *response, void *client_data)
{
  if (status != kLivoxLidarStatusSuccess)
  {
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr)
    return;

  uint8_t host_point_ipaddr[4]{0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;
  std::string readback;

  auto add = [&readback](const char *key, const std::string &value) {
    readback += std::string(readback.empty() ? "" : ", ") + "\"" + key + "\": " + value;
  };

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i)
  {
    LivoxLidarKeyValueParam *kv = (LivoxLidarKeyValueParam *)&response->data[off];
    if (kv->length == 0)   // the HAP pads the list with empty entries
      break;
    uint32_t number = 0;
    if (kv->length <= sizeof(number))
      memcpy(&number, &(kv->value[0]), kv->length);
    switch (kv->key)
    {
    case kKeyLidarPointDataHostIpCfg:
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
      break;
    case kKeyPclDataType:
      add("pcl_data_type", std::to_string(number));
      break;
    case kKeyPatternMode:
      add("pattern", std::to_string(number));
      break;
    case kKeyDualEmitEn:
      add("dual_emit", std::to_string(number));
      break;
    case kKeyBlindSpotSet:
      add("blind_spot_cm", std::to_string(number));
      break;
    case kKeyWorkMode:
      add("work_mode", std::to_string(number));
      break;
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  std::printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
              host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);
  std::fflush(stdout);
  write_report("{" + readback + "}");
}

// One callback per setting sent; once the lidar answered them all, read the settings back
void SettingCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data)
{
  const char *name = static_cast<const char *>(client_data);
  if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0)
  {
    std::string note = std::string(name) + " refused by the lidar (status " + std::to_string(status) +
                       ", code " + std::to_string(response ? response->ret_code : -1) + ")";
    std::fprintf(stderr, "WARNING: %s\n", note.c_str());
    std::lock_guard<std::mutex> lock(notes_mutex);
    notes.push_back(note);
  }
  if (--pending_sets == 0)
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

static void apply_settings(uint32_t handle)
{
  {
    std::lock_guard<std::mutex> lock(notes_mutex);
    notes.clear();
  }
  // Count first: a callback may come back before the next command is sent
  pending_sets = (settings.pattern >= 0) + (settings.dual_emit >= 0) + (settings.blind_spot_cm >= 0);
  if (pending_sets == 0)
  {
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }
  if (settings.pattern >= 0)
    SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(settings.pattern), SettingCallback,
                             const_cast<char *>("scan pattern"));
  if (settings.dual_emit >= 0)
    SetLivoxLidarDualEmit(handle, settings.dual_emit != 0, SettingCallback, const_cast<char *>("dual emit"));
  if (settings.blind_spot_cm >= 0)
    SetLivoxLidarBlindSpot(handle, static_cast<uint32_t>(settings.blind_spot_cm), SettingCallback,
                           const_cast<char *>("blind spot"));
}

// Parse the options after the positional arguments; false on a malformed one
static bool parse_options(int argc, const char *argv[], int first)
{
  for (int i = first; i < argc; i++)
  {
    const std::string option = argv[i];
    if (i + 1 >= argc)
      return false;
    const char *value = argv[++i];
    try
    {
      if (option == "--pattern")
        settings.pattern = std::stoi(value);
      else if (option == "--dual-emit")
        settings.dual_emit = std::stoi(value);
      else if (option == "--blind-spot")
        settings.blind_spot_cm = std::stol(value);
      else if (option == "--report")
        settings.report_path = value;
      else
        return false;
    }
    catch (const std::exception &)
    {
      return false;
    }
  }
  return true;
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo *info, void *client_data)
{
  if (info == nullptr)
  {
    std::printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  }
  // set the work mode to kLivoxLidarNormal, namely start the lidar
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
  apply_settings(handle);
}

int main(int argc, const char *argv[])
{
  const bool has_config = argc > 2 && std::strncmp(argv[2], "--", 2) != 0;
  if (argc < 2 || !parse_options(argc, argv, has_config ? 3 : 2))
  {
    std::fprintf(stderr, "Usage: %s <output_folder> [livox_config.json] [--pattern N] [--dual-emit 0|1] "
                         "[--blind-spot CM] [--report PATH]\n",
                 argv[0]);
    return 1;
  }
  const std::string folder = argv[1];
  const std::string config_path = has_config ? argv[2] : DEFAULT_CONFIG;

  if (!std::filesystem::is_directory(folder))
  {
    std::fprintf(stderr, "Output folder does not exist: %s\n", folder.c_str());
    return 1;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  writer = std::make_unique<H5PointWriter>(folder, "livox", MAX_POINTS_PER_FILE);

  if (!LivoxLidarSdkInit(config_path.c_str()))
  {
    std::fprintf(stderr, "Livox Init Failed (config %s)\n", config_path.c_str());
    LivoxLidarSdkUninit();
    return -1;
  }

  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  // One status line per second instead of per packet.
  auto last_report = std::chrono::steady_clock::now();
  uint64_t last_points = 0;
  while (!g_stop_requested)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto now = std::chrono::steady_clock::now();
    if (now - last_report >= std::chrono::seconds(1))
    {
      uint64_t total = points_received;
      std::printf("points/s: %llu\n", static_cast<unsigned long long>(total - last_points));
      std::fflush(stdout);
      last_points = total;
      last_report = now;
    }
  }

  std::printf("Stopping, flushing last chunk...\n");
  LivoxLidarSdkUninit();
  writer->close();
  std::printf("Done.\n");
  return 0;
}
