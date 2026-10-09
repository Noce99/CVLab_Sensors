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
// Usage: save_point_cloud_livox <output_folder> [livox_config.json]
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
#include <string>
#include <thread>

#include "h5_point_writer.hpp"

static const size_t MAX_POINTS_PER_FILE = 100000;
static const char *DEFAULT_CONFIG = "/home/ubm/CVLab_Sensors/get_data/config/livox_hap.json";

static std::unique_ptr<H5PointWriter> writer;

static std::atomic<uint64_t> points_received{0};
static std::atomic<int> last_time_type{-1};
static std::atomic<bool> warned_unsupported{false};

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

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i)
  {
    LivoxLidarKeyValueParam *kv = (LivoxLidarKeyValueParam *)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg)
    {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  std::printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
              host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);
  std::fflush(stdout);
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
  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

int main(int argc, const char *argv[])
{
  if (argc < 2)
  {
    std::fprintf(stderr, "Usage: %s <output_folder> [livox_config.json]\n", argv[0]);
    return 1;
  }
  const std::string folder = argv[1];
  const std::string config_path = argc > 2 ? argv[2] : DEFAULT_CONFIG;

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
