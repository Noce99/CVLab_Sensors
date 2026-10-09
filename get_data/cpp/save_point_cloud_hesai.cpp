// Usage: save_point_cloud_hesai <output_folder> [lidar_ip]
//
// Records the Hesai point cloud into H5 files (see h5_point_writer.hpp).
// Stop with SIGINT/SIGTERM: the last partial chunk is flushed before exiting.

#define NOMINMAX
#include "hesai_lidar_sdk.hpp"
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "h5_point_writer.hpp"

struct PointXYZIT
{
    PCL_ADD_POINT4D
    double timestamp;
    uint16_t ring;
    uint8_t intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(double, timestamp, timestamp)(std::uint16_t, ring, ring)(std::uint8_t, intensity, intensity))

static const size_t MAX_POINTS_PER_FILE = 300000;
static const char *DEFAULT_LIDAR_IP = "192.168.1.91"; // XT32

static std::unique_ptr<H5PointWriter> writer;

static std::atomic<uint64_t> frames_received{0};
static std::atomic<uint64_t> points_received{0};
static std::atomic<int64_t> last_frame_host_ns{0};

void lidarCallback(const LidarDecodedFrame<PointXYZIT> &frame)
{
    const int64_t host_ns = host_now_ns();
    if (frame.points_num == 0)
        return;

    // Hesai timestamps are seconds (double) in the lidar clock. The callback fires
    // once the frame is complete, so pair the host time with the frame end.
    writer->add_sync(std::llround(frame.frame_end_timestamp * 1e9), host_ns);

    for (uint32_t i = 0; i < frame.points_num; i++)
    {
        const PointXYZIT &p = frame.points[i];
        if (p.x == 0 && p.y == 0 && p.z == 0)
            continue;
        writer->add_point(p.x * 1000.0f, p.y * 1000.0f, p.z * 1000.0f,
                          std::llround(p.timestamp * 1e9), p.intensity);
    }

    frames_received++;
    points_received += frame.points_num;
    last_frame_host_ns = host_ns;
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::fprintf(stderr, "Usage: %s <output_folder> [lidar_ip]\n", argv[0]);
        return 1;
    }
    const std::string folder = argv[1];
    const std::string lidar_ip = argc > 2 ? argv[2] : DEFAULT_LIDAR_IP;

    if (!std::filesystem::is_directory(folder))
    {
        std::fprintf(stderr, "Output folder does not exist: %s\n", folder.c_str());
        return 1;
    }

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    writer = std::make_unique<H5PointWriter>(folder, "hesai", MAX_POINTS_PER_FILE);
    writer->set_time_source("hesai_lidar_clock");

    HesaiLidarSdk<PointXYZIT> sample;
    DriverParam param;
    param.use_gpu = false;
    param.input_param.source_type = DATA_FROM_LIDAR;
    param.input_param.device_ip_address = lidar_ip;
    param.input_param.ptc_port = 9347;   // lidar ptc port
    param.input_param.udp_port = 2369;   // point cloud destination port
    param.input_param.multicast_ip_address = "";

    param.input_param.use_ptc_connected = true; // true: use PTC connected, false: recv correction from local file
    param.input_param.correction_file_path = "/home/ubm/SDKs/HesaiLidar_SDK_2.0/correction/noce_correction_xt32/XT32_angle_correction.csv";
    param.input_param.firetimes_path = "/home/ubm/SDKs/HesaiLidar_SDK_2.0/correction/noce_correction_xt32/XT32_firetime_correction.csv";

    param.input_param.host_ip_address = "";      // point cloud destination ip, local ip
    param.input_param.fault_message_port = 9348; // fault message destination port

    param.decoder_param.enable_packet_loss_tool = false;
    param.decoder_param.socket_buffer_size = 262144000;

    sample.Init(param);
    sample.RegRecvCallback(lidarCallback);
    sample.Start();
    if (sample.lidar_ptr_->GetInitFinish(FailInit))
    {
        std::fprintf(stderr, "Hesai init failed\n");
        sample.Stop();
        return -1;
    }

    // One status line per second instead of per frame.
    auto last_report = std::chrono::steady_clock::now();
    uint64_t last_frames = 0, last_points = 0;
    while (!g_stop_requested)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto now = std::chrono::steady_clock::now();
        if (now - last_report >= std::chrono::seconds(1))
        {
            uint64_t frames = frames_received, points = points_received;
            int64_t silence_ms = last_frame_host_ns ? (host_now_ns() - last_frame_host_ns) / 1000000 : -1;
            std::printf("frames/s: %llu points/s: %llu ms since last frame: %lld\n",
                        static_cast<unsigned long long>(frames - last_frames),
                        static_cast<unsigned long long>(points - last_points),
                        static_cast<long long>(silence_ms));
            std::fflush(stdout);
            last_frames = frames;
            last_points = points;
            last_report = now;
        }
    }

    std::printf("Stopping, flushing last chunk...\n");
    sample.Stop();
    writer->close();
    std::printf("Done.\n");
    return 0;
}
