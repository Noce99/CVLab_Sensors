#define NOMINMAX
#include "hesai_lidar_sdk.hpp"
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <H5Cpp.h>

#define TIMESTAMP_PCL_STR (double, timestamp, timestamp)
#define RING_PCL_STR (std::uint16_t, ring, ring)
#define INTENSITY_PCL_STR (std::uint8_t, intensity, intensity)
#define CONFIDENCE_PCL_STR
#define WEIGHT_FACTOR_PCL_STR
#define ENV_LIGHT_PCL_STR

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
    (float, x, x)(float, y, y)(float, z, z)
        TIMESTAMP_PCL_STR
            RING_PCL_STR
                INTENSITY_PCL_STR
                    CONFIDENCE_PCL_STR
                        WEIGHT_FACTOR_PCL_STR
                            ENV_LIGHT_PCL_STR)

using namespace pcl::visualization;
std::shared_ptr<PCLVisualizer> pcl_viewer;
std::mutex mex_viewer;
uint32_t last_frame_time;
uint32_t cur_frame_time;
// log info, display frame message

int width = 500;
int height = 200;
int focal_x = 400;
int focal_y = 400;
double fx = focal_x;
double fy = focal_y;
double u_0 = height * 1.0 / 2;
double v_0 = width * 1.0 / 2;

cv::Mat depth_image(height, width, CV_8UC3);
std::vector<std::vector<double>> reflection_map(width, std::vector<double>(height, 0.0));

int fps = 20;
long int dt_microseconds = 1000 / fps;
long int last_start_microseconds = 0;

std::string FOLDER_TO_SAVE;

int MAX_POINTS = 100000 * 3; // 4 times more then livox
int h5_file_index = 0;
int buffer_index = 0;
std::shared_ptr<H5::H5File> h5file = nullptr;
std::shared_ptr<H5::DataSet> dataset = nullptr;
std::vector<int16_t> points_buffer(MAX_POINTS * 4); // x, y, z, timestamp
long int file_timestamp = 0;
long int relative_time_ns = 0;

void initialize_h5_file(const std::string &file_path)
{
    long int timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  
    std::string complete_file_path = file_path + "/" + std::to_string(timestamp) + ".h5";
    h5file = std::make_shared<H5::H5File>(complete_file_path, H5F_ACC_TRUNC);

    hsize_t dims[2] = {
        static_cast<hsize_t>(MAX_POINTS),
        static_cast<hsize_t>(4) // 4 attributes: x, y, z, timestamp
    };
    H5::DataSpace dataspace(2, dims);

    // H5::DSetCreatPropList prop;
    // hsize_t chunk_dims[2] = {1024, 4}; // Chunk size
    // prop.setChunk(2, chunk_dims);

    dataset = std::make_shared<H5::DataSet>(h5file->createDataSet("point_cloud", H5::PredType::NATIVE_INT16, dataspace)); // , prop));
    file_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    H5::Attribute timestamp_attr = dataset->createAttribute("file_creation_timestamp", H5::PredType::NATIVE_LONG, H5::DataSpace());
    timestamp_attr.write(H5::PredType::NATIVE_LONG, &file_timestamp);
    h5_file_index += 1;
    relative_time_ns = 0;
}

std::string millisecondsToDate(uint64_t ms)
{
    using namespace std::chrono;

    // Split into seconds and remaining milliseconds
    std::time_t seconds = static_cast<std::time_t>(ms / 1000);
    int millis = static_cast<int>(ms % 1000);

    // Format the date/time (C++20)
    std::string date_str = std::format("{:%Y-%m-%d_%H:%M:%S}", system_clock::from_time_t(seconds));

    // Append milliseconds
    date_str += std::format(".{:03d}", millis);

    return date_str;
}

cv::Mat image_from_reflection_map()
{
    double min_val = reflection_map[0][0];
    double max_val = reflection_map[0][0];

    max_val = 5.0; // Cap the max value for better visualization

    // Create OpenCV Mat for RGB image
    cv::Mat image(height, width, CV_8UC1, cv::Scalar(0));

    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            // Normalize to [0, 1]
            double normalized = (reflection_map[x][y] - min_val) / (max_val - min_val) * 255.0;

            // Apply inferno colormap
            // cv::Vec3b color = inferno_colormap(normalized);

            // Set pixel color (note: OpenCV uses BGR format)
            // clamp and cast to unsigned 8-bit value for CV_8UC1 image
            if (std::isnan(normalized) || std::isinf(normalized))
                normalized = 0.0;
            if (normalized < 0.0)
                normalized = 0.0;
            if (normalized > 255.0)
                normalized = 255.0;
            image.at<uchar>(y, x) = static_cast<uchar>(std::lround(normalized));
        }
    }

    return image;
}

void lidarCallback(const LidarDecodedFrame<PointXYZIT> &frame)
{
    cur_frame_time = GetMicroTickCount();
    if (cur_frame_time - last_frame_time > kMaxTimeInterval)
    {
        printf("Time between last frame and cur frame is: %u us\n", (cur_frame_time - last_frame_time));
    }
    last_frame_time = cur_frame_time;
    printf("frame:%d points:%u packet:%u start time:%lf end time:%lf\n", frame.frame_index, frame.points_num, frame.packet_num, frame.frame_start_timestamp, frame.frame_end_timestamp);
    pcl::PointCloud<PointXYZIT>::Ptr pcl_pointcloud(new pcl::PointCloud<PointXYZIT>);
    mex_viewer.lock();
    if (frame.points_num == 0)
        return;
    pcl_pointcloud->clear();
    pcl_pointcloud->resize(frame.points_num);
    pcl_pointcloud->points.assign(frame.points, frame.points + frame.points_num);
    pcl_pointcloud->height = 1;
    pcl_pointcloud->width = frame.points_num;
    pcl_pointcloud->is_dense = false;

    int time_step = static_cast<int>((frame.frame_end_timestamp - frame.frame_start_timestamp) / frame.points_num * 1e9);
    std::cout << "Time step: " << time_step << std::endl;

    for (const auto &point : *pcl_pointcloud)
    {
        // Access point data
        float x = point.z * 1000;
        float y = -point.x * 1000;
        float z = -point.y * 1000;
        double timestamp = point.timestamp;

        // std::cout << "Timestamp: " << timestamp << std::endl;

        if (x == 0 && y == 0 && z == 0)
        {
            continue;
        }
        if (buffer_index >= MAX_POINTS * 4)
        {
            // Save current buffer to H5 file
            dataset->write(points_buffer.data(), H5::PredType::NATIVE_INT16);
            h5file->close();

            // Initialize new H5 file
            initialize_h5_file(FOLDER_TO_SAVE);

            // Reset buffer index and add current point
            buffer_index = 0;
        }
        points_buffer[buffer_index++] = x;
        points_buffer[buffer_index++] = y;
        points_buffer[buffer_index++] = z;
        points_buffer[buffer_index++] = relative_time_ns;

        relative_time_ns += time_step;
    }
    mex_viewer.unlock();
}

int main(int argc, char *argv[])
{

    if (argc < 2)
    {
        return 1;
    }
    else
    {
        FOLDER_TO_SAVE = argv[1];
    }

    // std::cout << "Saving data to folder: " << FOLDER_TO_SAVE << std::endl;
    // if (!std::filesystem::exists(FOLDER_TO_SAVE)) {
    //   std::filesystem::create_directories(FOLDER_TO_SAVE);
    // }
    // if (!std::filesystem::exists(FOLDER_TO_SAVE + "/lidar")) {
    //   std::filesystem::create_directories(FOLDER_TO_SAVE + "/lidar");
    // }

    std::string json_setup_path = "/home/ubm/SDKs/Livox-SDK2/samples/livox_lidar_quick_start/hap_noce.json";

    // Initalize H5 File
    initialize_h5_file(FOLDER_TO_SAVE);

    HesaiLidarSdk<PointXYZIT> sample;
    DriverParam param;
    // assign param
    param.use_gpu = (argc > 1);
    // assign param
    param.input_param.source_type = DATA_FROM_LIDAR;
    param.input_param.device_ip_address = "192.168.1.91"; // lidar ip
    param.input_param.ptc_port = 9347;                    // lidar ptc port
    param.input_param.udp_port = 2369;                    // point cloud destination port
    param.input_param.multicast_ip_address = "";

    param.input_param.use_ptc_connected = true; // true: use PTC connected, false: recv correction from local file
    param.input_param.correction_file_path = "/home/ubm/SDKs/HesaiLidar_SDK_2.0/correction/noce_correction_xt32/XT32_angle_correction.csv";
    param.input_param.firetimes_path = "/home/ubm/SDKs/HesaiLidar_SDK_2.0/correction/noce_correction_xt32/XT32_firetime_correction.csv";

    param.input_param.host_ip_address = "";      // point cloud destination ip, local ip
    param.input_param.fault_message_port = 9348; // fault message destination port

    param.decoder_param.enable_packet_loss_tool = false;
    param.decoder_param.socket_buffer_size = 262144000;

    // init lidar with param
    sample.Init(param);

    // assign callback fuction
    sample.RegRecvCallback(lidarCallback);

    // star process thread
    last_frame_time = GetMicroTickCount();
    sample.Start();
    if (sample.lidar_ptr_->GetInitFinish(FailInit))
    {
        sample.Stop();
        return -1;
    }

    while (1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    return 0;
}