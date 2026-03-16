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

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <format>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <H5Cpp.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

uint64_t points = 0;
uint64_t first_timestamp = 0;

int width = 1000;
int height = 400;
int focal_x = 800;
int focal_y = 800;
cv::Mat depth_image(height, width, CV_64F);
std::vector<std::vector<double>> reflection_map(width, std::vector<double>(height, 0.0));

int fps = 1;
long int dt_microseconds = 10000000; // 1000 / fps;
long int last_start_microseconds = 0;

std::string FOLDER_TO_SAVE;
std::string next_name = "not_set";

int MAX_POINTS = 100000*100;
int h5_file_index = 0;
int buffer_index = 0;
std::shared_ptr<H5::H5File> h5file = nullptr;
std::shared_ptr<H5::DataSet> dataset = nullptr;
std::vector<int16_t> points_buffer(MAX_POINTS * 5);
long int file_timestamp = 0;
long int relative_time_ns = 0;

using namespace boost::interprocess;

std::string millisecondsToDate(uint64_t ms) {
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

cv::Mat image_from_reflection_map(){
  double min_val = reflection_map[0][0];
  double max_val = reflection_map[0][0];

  min_val = 0.0;
  max_val = 5.0; // Cap the max value for better visualization

  // Create OpenCV Mat for RGB image
  cv::Mat image(height, width, CV_8UC1, cv::Scalar(0));

  for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
          // Normalize to [0, 1]
          double normalized = (reflection_map[x][y] - min_val) / (max_val - min_val) * 255.0;
          
          // Apply inferno colormap
          // cv::Vec3b color = inferno_colormap(normalized);
          
          // Set pixel color (note: OpenCV uses BGR format)
            // clamp and cast to unsigned 8-bit value for CV_8UC1 image
            if (std::isnan(normalized) || std::isinf(normalized)) normalized = 0.0;
            if (normalized < 0.0) normalized = 0.0;
            if (normalized > 255.0) normalized = 255.0;
            image.at<uchar>(y, x) = static_cast<uchar>(std::lround(normalized));
      }
  }

  return image;
}

void initialize_h5_file(const std::string &file_path)
{
  long int timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  next_name = std::to_string(timestamp);

  std::string complete_file_path = file_path + "/" + std::to_string(timestamp) + ".h5";
  h5file = std::make_shared<H5::H5File>(complete_file_path, H5F_ACC_TRUNC);

  hsize_t dims[2] = {
      static_cast<hsize_t>(MAX_POINTS),
      static_cast<hsize_t>(5) // 4 attributes: x, y, z, reflectivity, timestamp
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


void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }

  points++;


  uint64_t timestamp = 0;
  for(int i = 0; i < 8; i++) {
      timestamp |= static_cast<uint64_t>(data->timestamp[i]) << (8 * i);
  }

  // std::cout << "timestamp = [" << millisecondsToDate(timestamp / 1000) << "]" << "[" << data->time_type <<"]"<< std::endl;


  if (first_timestamp == 0){
    first_timestamp = timestamp;
  }
  
  double fx = focal_x;
  double fy = focal_y;
  double u_0 = height*1.0 / 2;
  double v_0 = width*1.0 / 2;

  int time_step = static_cast<int>(static_cast<double>(data->time_interval) / static_cast<double>(data->dot_num) * 0.1 * 1e3); // in microseconds

  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++) {
      //p_point_data[i].x;
      //p_point_data[i].y;
      //p_point_data[i].z;
      double x = p_point_data[i].z;
      double y = p_point_data[i].y;
      double z = p_point_data[i].x;
      if (z > 0){
        double reflectivity = p_point_data[i].reflectivity;
        
        int u = int(x*fx/z + u_0);
        int v = int(y*fy/z + v_0);


        if (v > 0 && v < width && u > 0 && u < height){
          depth_image.at<double>(height-u, width-v) = z;
          if (reflection_map[width-v][height-u] > 0){
            reflection_map[width-v][height-u] = (reflection_map[width-v][height-u] + reflectivity) / 2.0;
          }else{
            reflection_map[width-v][height-u] = reflectivity;
          }
        }

        points_buffer[buffer_index++] = x;
        points_buffer[buffer_index++] = y;
        points_buffer[buffer_index++] = z;
        points_buffer[buffer_index++] = reflectivity;
        points_buffer[buffer_index++] = relative_time_ns;

        relative_time_ns += time_step;
      }
    }

    long int current_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    if (last_start_microseconds == 0) {
        last_start_microseconds = current_microseconds;
    }else if (current_microseconds - last_start_microseconds > dt_microseconds) {
        std::string reflection_filename = next_name + ".png";
        std::string path_to_save = FOLDER_TO_SAVE + "/lidar/" + reflection_filename;

        std::cout << "Saving images... [" << millisecondsToDate(current_microseconds / 1000) << "]" << std::endl;
        std::cout << "In " << path_to_save << std::endl;

        // std::string depth_filename = "depth_map_" + std::to_string(current_milliseconds) + ".png";
        // cv::imwrite(depth_filename, depth_image);
        cv::imwrite(path_to_save, image_from_reflection_map());

        last_start_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

        depth_image.setTo(cv::Scalar(0));
        for (auto& row : reflection_map) {
          std::fill(row.begin(), row.end(), 0.0);
        }

        // Save current buffer to H5 file
        dataset->write(points_buffer.data(), H5::PredType::NATIVE_INT16);
        h5file->close();

        // Initialize new H5 file
        initialize_h5_file(FOLDER_TO_SAVE + "/lidar_raw");

        // Reset buffer index and add current point
        buffer_index = 0;
    }

  }else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
    std::cout << "DUE" << std::endl;
  } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
    LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;
    std::cout << "TRE" << std::endl;
  }
}

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  // printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
  //     status, handle, response->ret_code, response->error_key);

}
void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
  LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    // printf("Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    return;
  }

  uint8_t host_point_ipaddr[4] {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;

  uint8_t host_imu_ipaddr[4] {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
      host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

  printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
    host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  // printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  // set the work mode to kLivoxLidarNormal, namely start the lidar
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}


int main(int argc, const char *argv[]) {
  std::cout << "argc: " << argc << std::endl;

  if (argc != 2) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y_%m_%d_%H_%M_%S");
    FOLDER_TO_SAVE = "./calibration_data/" + ss.str();
  }else{
    FOLDER_TO_SAVE = argv[1];
  }

  std::cout << "Saving data to folder: " << FOLDER_TO_SAVE << std::endl;
  if (!std::filesystem::exists(FOLDER_TO_SAVE)) {
    std::filesystem::create_directories(FOLDER_TO_SAVE);
  }
  if (!std::filesystem::exists(FOLDER_TO_SAVE + "/lidar")) {
    std::filesystem::create_directories(FOLDER_TO_SAVE + "/lidar");
  }
  if (!std::filesystem::exists(FOLDER_TO_SAVE + "/lidar_raw")) {
    std::filesystem::create_directories(FOLDER_TO_SAVE + "/lidar_raw");
  }

  std::string json_setup_path = "/home/ubm/SDKs/Livox-SDK2/samples/livox_lidar_quick_start/hap_noce.json";

  initialize_h5_file(FOLDER_TO_SAVE + "/lidar_raw");

  // REQUIRED, to init Livox SDK2
  if (!LivoxLidarSdkInit(json_setup_path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }
  
  // REQUIRED, to get point cloud data via 'PointCloudCallback'
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  
  // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  while (true){}
  return 0;
}
