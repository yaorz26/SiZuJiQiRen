#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

struct PointXYZ {
  float x;
  float y;
  float z;
};

class Go2LidarSnapshot : public rclcpp::Node {
public:
  Go2LidarSnapshot() : Node("go2_lidar_snapshot") {
    topic_ = this->declare_parameter<std::string>("topic", "/utlidar/cloud_deskewed");
    duration_s_ = this->declare_parameter<double>("duration_s", 3.0);
    max_points_ = this->declare_parameter<int>("max_points", 120000);
    const int declared_sample_stride =
      static_cast<int>(this->declare_parameter<int>("sample_stride", 1));
    sample_stride_ = std::max(1, declared_sample_stride);
    output_prefix_ = this->declare_parameter<std::string>("output_prefix", "go2_lidar");

    if (duration_s_ <= 0.0) {
      throw std::runtime_error("duration_s must be positive");
    }
    if (max_points_ <= 0) {
      throw std::runtime_error("max_points must be positive");
    }

    start_time_ = this->now();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_,
      10,
      std::bind(&Go2LidarSnapshot::on_cloud, this, std::placeholders::_1)
    );
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&Go2LidarSnapshot::on_timer, this)
    );

    RCLCPP_INFO(this->get_logger(), "Subscribing lidar topic: %s", topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Capture duration=%.1fs max_points=%d sample_stride=%d output_prefix=%s",
      duration_s_,
      max_points_,
      sample_stride_,
      output_prefix_.c_str()
    );
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string topic_;
  std::string output_prefix_;
  double duration_s_ = 3.0;
  int max_points_ = 120000;
  int sample_stride_ = 1;
  rclcpp::Time start_time_;
  bool done_ = false;
  std::size_t frames_ = 0;
  std::vector<PointXYZ> points_;

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (done_) {
      return;
    }
    frames_ += 1;

    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    for (const auto & field : msg->fields) {
      if (field.name == "x") {
        x_offset = static_cast<int>(field.offset);
      } else if (field.name == "y") {
        y_offset = static_cast<int>(field.offset);
      } else if (field.name == "z") {
        z_offset = static_cast<int>(field.offset);
      }
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      RCLCPP_ERROR(this->get_logger(), "PointCloud2 does not contain x/y/z fields.");
      finish();
      return;
    }

    const std::size_t count_before = points_.size();
    const std::size_t point_count = static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    for (std::size_t i = 0; i < point_count; ++i) {
      if (static_cast<int>(i % static_cast<std::size_t>(sample_stride_)) != 0) {
        continue;
      }
      const std::size_t base = static_cast<std::size_t>(msg->point_step) * i;
      if (base + msg->point_step > msg->data.size()) {
        break;
      }

      PointXYZ point{
        read_float32(msg->data, base + static_cast<std::size_t>(x_offset)),
        read_float32(msg->data, base + static_cast<std::size_t>(y_offset)),
        read_float32(msg->data, base + static_cast<std::size_t>(z_offset)),
      };
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        points_.push_back(point);
      }
      if (points_.size() >= static_cast<std::size_t>(max_points_)) {
        break;
      }
    }

    if (frames_ % 5 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "frames=%zu total_points=%zu last_added=%zu",
        frames_,
        points_.size(),
        points_.size() - count_before
      );
    }

    if (points_.size() >= static_cast<std::size_t>(max_points_)) {
      finish();
    }
  }

  void on_timer() {
    if (done_) {
      return;
    }
    const double elapsed = (this->now() - start_time_).seconds();
    if (elapsed >= duration_s_) {
      finish();
    }
  }

  static float read_float32(const std::vector<unsigned char> & data, std::size_t offset) {
    float value = 0.0f;
    std::memcpy(&value, data.data() + offset, sizeof(float));
    return value;
  }

  void finish() {
    if (done_) {
      return;
    }
    done_ = true;
    timer_->cancel();

    if (points_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No points captured. Check lidar topic and publisher status.");
      rclcpp::shutdown();
      return;
    }

    const std::string csv_path = output_prefix_ + "_points.csv";
    const std::string ply_path = output_prefix_ + "_points.ply";
    write_csv(csv_path);
    write_ply(ply_path);

    PointXYZ min_point{
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(),
    };
    PointXYZ max_point{
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::lowest(),
    };
    for (const auto & point : points_) {
      min_point.x = std::min(min_point.x, point.x);
      min_point.y = std::min(min_point.y, point.y);
      min_point.z = std::min(min_point.z, point.z);
      max_point.x = std::max(max_point.x, point.x);
      max_point.y = std::max(max_point.y, point.y);
      max_point.z = std::max(max_point.z, point.z);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Saved lidar snapshot: %s, %s",
      csv_path.c_str(),
      ply_path.c_str()
    );
    RCLCPP_INFO(
      this->get_logger(),
      "frames=%zu points=%zu x=[%.2f,%.2f] y=[%.2f,%.2f] z=[%.2f,%.2f]",
      frames_,
      points_.size(),
      min_point.x,
      max_point.x,
      min_point.y,
      max_point.y,
      min_point.z,
      max_point.z
    );

    rclcpp::shutdown();
  }

  void write_csv(const std::string & path) const {
    std::ofstream file(path, std::ios::out | std::ios::trunc);
    file << "x,y,z\n";
    for (const auto & point : points_) {
      file << point.x << "," << point.y << "," << point.z << "\n";
    }
  }

  void write_ply(const std::string & path) const {
    std::ofstream file(path, std::ios::out | std::ios::trunc);
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << points_.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
    for (const auto & point : points_) {
      file << point.x << " " << point.y << " " << point.z << "\n";
    }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2LidarSnapshot>());
  rclcpp::shutdown();
  return 0;
}
