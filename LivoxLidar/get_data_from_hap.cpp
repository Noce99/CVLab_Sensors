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

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

uint64_t points = 0;
uint64_t first_timestamp = 0;

int width = 800;
int height = 400;
int focal_x = 200;
int focal_y = 200;
std::vector<std::vector<double>> depth_map(width, std::vector<double>(height, 0.0));
std::vector<std::vector<double>> reflection_map(width, std::vector<double>(height, 0.0));

using namespace boost::interprocess;

struct SharedImageHeader {
    int rows;
    int cols;
    int channels;
};

// Define image dimensions
const char* shm_name_depth_map = "DepthMap";
const char* shm_name_reflection_map = "ReflectionMap";
const int rows = height;
const int cols = width;
const int channels = 3;
const size_t img_size = static_cast<size_t>(rows) * cols * channels;
const size_t total_size = sizeof(SharedImageHeader) + img_size;
uchar* depth_map_data_ptr;
uchar* reflection_map_data_ptr;
shared_memory_object* depth_shm_ptr = nullptr;
shared_memory_object* reflection_shm_ptr = nullptr;
mapped_region* depth_region_ptr = nullptr;
mapped_region* reflection_region_ptr = nullptr;

void initialize_shared_memory(){
    shared_memory_object::remove(shm_name_depth_map);
    shared_memory_object::remove(shm_name_reflection_map);

    depth_shm_ptr = new shared_memory_object(create_only, shm_name_depth_map, read_write);
    depth_shm_ptr->truncate(total_size);
    reflection_shm_ptr = new shared_memory_object(create_only, shm_name_reflection_map, read_write);
    reflection_shm_ptr->truncate(total_size);

    depth_region_ptr = new mapped_region(*depth_shm_ptr, read_write);
    void* depth_addr = depth_region_ptr->get_address();
    reflection_region_ptr = new mapped_region(*reflection_shm_ptr, read_write);
    void* reflection_addr = reflection_region_ptr->get_address();

    auto* depth_header = reinterpret_cast<SharedImageHeader*>(depth_addr);
    depth_header->rows = rows;
    depth_header->cols = cols;
    depth_header->channels = channels;
    auto* reflection_header = reinterpret_cast<SharedImageHeader*>(depth_addr);
    reflection_header->rows = rows;
    reflection_header->cols = cols;
    reflection_header->channels = channels;

    depth_map_data_ptr = reinterpret_cast<uchar*>(depth_addr) + sizeof(SharedImageHeader);
    reflection_map_data_ptr = reinterpret_cast<uchar*>(reflection_addr) + sizeof(SharedImageHeader);
}


std::string nanosecondsToDate(uint64_t ns) {

    std::chrono::nanoseconds ns_since_epoch = std::chrono::nanoseconds(ns);

    using namespace std::chrono;
    
    // Convert to system_clock time_point
    auto sys_time = time_point<system_clock>(duration_cast<system_clock::duration>(ns_since_epoch));
    
    // Convert to time_t for formatting
    auto time_t_value = system_clock::to_time_t(sys_time);
    
    // Get remaining nanoseconds
    auto remaining_ns = ns_since_epoch - duration_cast<nanoseconds>(
        system_clock::duration(time_t_value * 1000000000LL));
    
    // Format the date (C++20)
    std::string date_str = std::format("{:%Y-%m-%d %H:%M:%S}", 
                                      std::chrono::system_clock::from_time_t(time_t_value));
    
    // Add nanoseconds
    date_str += std::format(".{:09d}", remaining_ns.count());
    
    return date_str;
}

cv::Mat image_from_depth_map(){
  double min_val = 0;
  double max_val = 10000;

  // Create OpenCV Mat for RGB image
  cv::Mat image(height, width, CV_8UC3);

  // Inferno colormap function
  auto inferno_colormap = [](double value) -> cv::Vec3b {
      // Clamp value to [0, 1]
      value = 1 - std::max(0.0, std::min(1.0, value));
      
      // Inferno colormap approximation (simplified version)
      double r, g, b;
      
      if (value < 0.25) {
          // Dark purple to purple
          double t = value / 0.25;
          r = 0.1098 * t;
          g = 0.0275 * t;
          b = 0.3020 * t;
      } else if (value < 0.5) {
          // Purple to red
          double t = (value - 0.25) / 0.25;
          r = 0.1098 + (0.8392 - 0.1098) * t;
          g = 0.0275 + (0.0000 - 0.0275) * t;
          b = 0.3020 + (0.0000 - 0.3020) * t;
      } else if (value < 0.75) {
          // Red to yellow
          double t = (value - 0.5) / 0.25;
          r = 0.8392 + (0.9922 - 0.8392) * t;
          g = 0.0000 + (0.6824 - 0.0000) * t;
          b = 0.0000;
      } else {
          // Yellow to white
          double t = (value - 0.75) / 0.25;
          r = 0.9922 + (0.9882 - 0.9922) * t;
          g = 0.6824 + (0.9490 - 0.6824) * t;
          b = 0.0000 + (0.6314 - 0.0000) * t;
      }
      
      return cv::Vec3b(static_cast<uchar>(b * 255), 
                      static_cast<uchar>(g * 255), 
                      static_cast<uchar>(r * 255));
  };

  for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
          // Normalize to [0, 1]
          double normalized = (depth_map[x][y] - min_val) / (max_val - min_val);
          
          // Apply inferno colormap
          cv::Vec3b color = inferno_colormap(normalized);
          
          // Set pixel color (note: OpenCV uses BGR format)
          image.at<cv::Vec3b>(y, x) = color;
      }
  }

  return image;
}

cv::Mat image_from_reflection_map(){
  double min_val = 0;
  double max_val = 255;

  // Create OpenCV Mat for RGB image
  cv::Mat image(height, width, CV_8UC3);

  // Inferno colormap function
  auto inferno_colormap = [](double value) -> cv::Vec3b {
      // Clamp value to [0, 1]
      value = 1 - std::max(0.0, std::min(1.0, value));
      
      // Inferno colormap approximation (simplified version)
      double r, g, b;
      
      if (value < 0.25) {
          // Dark purple to purple
          double t = value / 0.25;
          r = 0.1098 * t;
          g = 0.0275 * t;
          b = 0.3020 * t;
      } else if (value < 0.5) {
          // Purple to red
          double t = (value - 0.25) / 0.25;
          r = 0.1098 + (0.8392 - 0.1098) * t;
          g = 0.0275 + (0.0000 - 0.0275) * t;
          b = 0.3020 + (0.0000 - 0.3020) * t;
      } else if (value < 0.75) {
          // Red to yellow
          double t = (value - 0.5) / 0.25;
          r = 0.8392 + (0.9922 - 0.8392) * t;
          g = 0.0000 + (0.6824 - 0.0000) * t;
          b = 0.0000;
      } else {
          // Yellow to white
          double t = (value - 0.75) / 0.25;
          r = 0.9922 + (0.9882 - 0.9922) * t;
          g = 0.6824 + (0.9490 - 0.6824) * t;
          b = 0.0000 + (0.6314 - 0.0000) * t;
      }
      
      return cv::Vec3b(static_cast<uchar>(b * 255), 
                      static_cast<uchar>(g * 255), 
                      static_cast<uchar>(r * 255));
  };

  for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
          // Normalize to [0, 1]
          double normalized = (reflection_map[x][y] - min_val) / (max_val - min_val);
          
          // Apply inferno colormap
          cv::Vec3b color = inferno_colormap(normalized);
          
          // Set pixel color (note: OpenCV uses BGR format)
          image.at<cv::Vec3b>(y, x) = color;
      }
  }

  return image;
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

  if (first_timestamp == 0){
    first_timestamp = timestamp;
  }
  
  double fx = focal_x;
  double fy = focal_y;
  double u_0 = height*1.0 / 2;
  double v_0 = width*1.0 / 2;


  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++) {
      //p_point_data[i].x;
      //p_point_data[i].y;
      //p_point_data[i].z;
      double x = p_point_data[i].z;
      double y = p_point_data[i].y;
      double z = p_point_data[i].x;
      double reflectivity = p_point_data[i].reflectivity;
      
      int u = int(x*fx/z + u_0);
      int v = int(y*fy/z + v_0);


      if (v > 0 && v < width && u > 0 && u < height){
        depth_map[width-v][height-u] = z;
        reflection_map[width-v][height-u] = reflectivity;
      }
    }

    if (points % 1000 == 0){
      std::memcpy(depth_map_data_ptr, image_from_depth_map().data, img_size);
      std::memcpy(reflection_map_data_ptr, image_from_reflection_map().data, img_size);
      for (auto& row : depth_map) {
        std::fill(row.begin(), row.end(), 0.0);
      }
      for (auto& row : reflection_map) {
        std::fill(row.begin(), row.end(), 0.0);
      }
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
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

}
void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
  LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("Query lidar internal info failed.\n");
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
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  // set the work mode to kLivoxLidarNormal, namely start the lidar
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}


int main(int argc, const char *argv[]) {
  std::string path;
  if (argc != 2) {
    path = "/home/ubm/SDKs/Livox-SDK2/samples/livox_lidar_quick_start/hap_noce.json";
  }else{
    path = argv[1];
  }

  initialize_shared_memory();

  // REQUIRED, to init Livox SDK2
  if (!LivoxLidarSdkInit(path.c_str())) {
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
