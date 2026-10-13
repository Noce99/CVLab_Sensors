// Shared HDF5 writer for the lidar recorders (Livox and Hesai).
//
// File layout (format_version = 2), one file per chunk of points:
//   xyz           float32 (N, 3)  native sensor coordinates, millimetres
//   timestamp_ns  int64   (N,)    per-point time, lidar clock, nanoseconds
//   intensity     uint8   (N,)    reflectivity / intensity
//   sync          int64   (M, 2)  [lidar_ns, host_ns] pairs, one per packet/frame,
//                                 to map the lidar clock onto the host clock
//   attrs: format_version, lidar, time_source, coordinates
//
// Files are named <host time of first point, us>.h5 (lidar time mapped through
// the latest sync pair, so it is approximate: use timestamp_ns + sync for precise work). They are written to
// "<name>.h5.tmp" and renamed once complete, so readers never see a partial file.
// Writing happens on a background thread so the SDK callback never blocks on disk.

#pragma once

#include <H5Cpp.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

inline int64_t host_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

class H5PointWriter
{
public:
  H5PointWriter(std::string folder, std::string lidar_name, size_t max_points)
      : folder_(std::move(folder)), lidar_name_(std::move(lidar_name)), max_points_(max_points)
  {
    current_.reserve(max_points_);
    writer_ = std::thread(&H5PointWriter::writer_loop, this);
  }

  ~H5PointWriter() { close(); }

  // Called from the SDK callback thread.
  void add_point(float x, float y, float z, int64_t t_lidar_ns, uint8_t intensity)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_.n == 0)
      current_.first_host_ns = has_offset_ ? t_lidar_ns + host_minus_lidar_ns_ : host_now_ns();
    current_.push(x, y, z, t_lidar_ns, intensity);
    if (current_.n >= max_points_)
      rotate_locked();
  }

  // One [lidar_ns, host_ns] pair per received packet/frame.
  void add_sync(int64_t t_lidar_ns, int64_t t_host_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_.sync.push_back(t_lidar_ns);
    current_.sync.push_back(t_host_ns);
    host_minus_lidar_ns_ = t_host_ns - t_lidar_ns;
    has_offset_ = true;
  }

  void set_time_source(const std::string &time_source)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    time_source_ = time_source;
  }

  // Flush the partial chunk and wait for every pending file to be on disk.
  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_)
        return;
      closed_ = true;
      if (current_.n > 0)
        rotate_locked();
    }
    cv_.notify_all();
    if (writer_.joinable())
      writer_.join();
  }

private:
  struct Chunk
  {
    size_t n = 0;
    int64_t first_host_ns = 0;
    std::string time_source;
    std::vector<float> xyz;
    std::vector<int64_t> t;
    std::vector<uint8_t> intensity;
    std::vector<int64_t> sync;

    void reserve(size_t max_points)
    {
      xyz.reserve(max_points * 3);
      t.reserve(max_points);
      intensity.reserve(max_points);
    }

    void push(float x, float y, float z, int64_t ts, uint8_t i)
    {
      xyz.push_back(x);
      xyz.push_back(y);
      xyz.push_back(z);
      t.push_back(ts);
      intensity.push_back(i);
      n++;
    }
  };

  void rotate_locked()
  {
    current_.time_source = time_source_;
    pending_.push_back(std::move(current_));
    current_ = Chunk();
    current_.reserve(max_points_);
    cv_.notify_one();
  }

  void writer_loop()
  {
    while (true)
    {
      Chunk chunk;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !pending_.empty() || closed_; });
        if (pending_.empty())
          return; // closed and drained
        chunk = std::move(pending_.front());
        pending_.pop_front();
      }
      try
      {
        write_chunk(chunk);
      }
      catch (const H5::Exception &e)
      {
        std::fprintf(stderr, "[%s] ERROR writing H5 chunk (%zu points lost): %s\n",
                     lidar_name_.c_str(), chunk.n, e.getCDetailMsg());
      }
      catch (const std::exception &e)
      {
        std::fprintf(stderr, "[%s] ERROR writing H5 chunk (%zu points lost): %s\n",
                     lidar_name_.c_str(), chunk.n, e.what());
      }
    }
  }

  void write_chunk(const Chunk &c)
  {
    const std::string name = std::to_string(c.first_host_ns / 1000) + ".h5";
    const std::filesystem::path final_path = std::filesystem::path(folder_) / name;
    const std::filesystem::path tmp_path = final_path.string() + ".tmp";

    // Never leave a partial file behind (e.g. disk full)
    struct RemoveOnFailure
    {
      const std::filesystem::path &path;
      bool armed = true;
      ~RemoveOnFailure()
      {
        std::error_code ec;
        if (armed)
          std::filesystem::remove(path, ec);
      }
    } cleanup{tmp_path};

    {
      H5::H5File file(tmp_path.string(), H5F_ACC_TRUNC);

      hsize_t dims_xyz[2] = {c.n, 3};
      H5::DataSpace space_xyz(2, dims_xyz);
      file.createDataSet("xyz", H5::PredType::NATIVE_FLOAT, space_xyz)
          .write(c.xyz.data(), H5::PredType::NATIVE_FLOAT);

      hsize_t dims_n[1] = {c.n};
      H5::DataSpace space_n(1, dims_n);
      file.createDataSet("timestamp_ns", H5::PredType::NATIVE_INT64, space_n)
          .write(c.t.data(), H5::PredType::NATIVE_INT64);
      file.createDataSet("intensity", H5::PredType::NATIVE_UINT8, space_n)
          .write(c.intensity.data(), H5::PredType::NATIVE_UINT8);

      hsize_t dims_sync[2] = {c.sync.size() / 2, 2};
      H5::DataSpace space_sync(2, dims_sync);
      H5::DataSet sync = file.createDataSet("sync", H5::PredType::NATIVE_INT64, space_sync);
      if (!c.sync.empty())
        sync.write(c.sync.data(), H5::PredType::NATIVE_INT64);

      H5::DataSpace scalar;
      H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
      int version = 2;
      file.createAttribute("format_version", H5::PredType::NATIVE_INT, scalar)
          .write(H5::PredType::NATIVE_INT, &version);
      file.createAttribute("lidar", str_type, scalar).write(str_type, lidar_name_);
      file.createAttribute("time_source", str_type, scalar).write(str_type, c.time_source);
      file.createAttribute("coordinates", str_type, scalar).write(str_type, std::string("native sensor frame, mm"));
    }

    std::filesystem::rename(tmp_path, final_path);
    cleanup.armed = false;
  }

  std::string folder_;
  std::string lidar_name_;
  size_t max_points_;
  std::string time_source_ = "unknown";
  int64_t host_minus_lidar_ns_ = 0; // latest sync offset, used to name files in host time
  bool has_offset_ = false;

  std::mutex mutex_;
  std::condition_variable cv_;
  Chunk current_;
  std::deque<Chunk> pending_;
  bool closed_ = false;
  std::thread writer_;
};

// SIGINT/SIGTERM -> request a clean shutdown; main() polls this flag.
inline std::atomic<bool> g_stop_requested{false};

inline void request_stop(int) { g_stop_requested = true; }
