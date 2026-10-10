#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

struct PointXYZ {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

class Go2CloudMapProjector : public rclcpp::Node {
public:
  Go2CloudMapProjector() : Node("go2_cloud_map_projector") {
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/utlidar/cloud_deskewed");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map_from_cloud");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");
    resolution_ = this->declare_parameter<double>("resolution", 0.05);
    min_z_ = this->declare_parameter<double>("min_z", -1.00);
    max_z_ = this->declare_parameter<double>("max_z", 2.00);
    min_range_ = this->declare_parameter<double>("min_range", 0.20);
    max_range_ = this->declare_parameter<double>("max_range", 30.0);
    padding_ = this->declare_parameter<double>("padding", 1.0);
    occupied_threshold_ = this->declare_parameter<int>("occupied_threshold", 1);
    publish_period_ = this->declare_parameter<double>("publish_period", 1.0);
    max_points_per_cloud_ = this->declare_parameter<int>("max_points_per_cloud", 500000);
    max_cells_ = this->declare_parameter<int>("max_cells", 2000000);

    if (resolution_ <= 0.0) {
      throw std::runtime_error("resolution must be positive");
    }
    if (min_z_ > max_z_) {
      throw std::runtime_error("min_z must be <= max_z");
    }
    if (max_points_per_cloud_ <= 0) {
      throw std::runtime_error("max_points_per_cloud must be positive");
    }

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      rclcpp::QoS(1).transient_local().reliable()
    );
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Go2CloudMapProjector::on_cloud, this, std::placeholders::_1)
    );
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period_)),
      std::bind(&Go2CloudMapProjector::publish_map, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Projecting %s to %s, frame=%s resolution=%.3f z=[%.2f, %.2f]",
      cloud_topic_.c_str(),
      map_topic_.c_str(),
      frame_id_.c_str(),
      resolution_,
      min_z_,
      max_z_
    );
  }

private:
  std::string cloud_topic_;
  std::string map_topic_;
  std::string frame_id_;
  double resolution_ = 0.05;
  double min_z_ = 0.10;
  double max_z_ = 1.60;
  double min_range_ = 0.20;
  double max_range_ = 30.0;
  double padding_ = 1.0;
  int occupied_threshold_ = 1;
  double publish_period_ = 1.0;
  int max_points_per_cloud_ = 500000;
  int max_cells_ = 2000000;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::int64_t, std::uint16_t> cell_counts_;
  std_msgs::msg::Header latest_header_;
  bool has_cloud_ = false;
  bool has_cells_ = false;
  int min_ix_ = 0;
  int max_ix_ = 0;
  int min_iy_ = 0;
  int max_iy_ = 0;

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "PointCloud2 does not contain x/y/z fields"
      );
      return;
    }

    latest_header_ = msg->header;
    const std::size_t point_count =
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    double raw_min_z = std::numeric_limits<double>::max();
    double raw_max_z = std::numeric_limits<double>::lowest();
    std::size_t finite_points = 0;
    std::size_t accepted_points = 0;

    for (std::size_t i = 0; i < point_count; ++i) {
      const std::size_t base = static_cast<std::size_t>(msg->point_step) * i;
      if (base + msg->point_step > msg->data.size()) {
        break;
      }

      PointXYZ point{
        read_float32(msg->data, base + static_cast<std::size_t>(x_offset)),
        read_float32(msg->data, base + static_cast<std::size_t>(y_offset)),
        read_float32(msg->data, base + static_cast<std::size_t>(z_offset)),
      };
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      finite_points += 1;
      raw_min_z = std::min(raw_min_z, static_cast<double>(point.z));
      raw_max_z = std::max(raw_max_z, static_cast<double>(point.z));

      if (point.z < min_z_ || point.z > max_z_) {
        continue;
      }
      if (max_range_ > 0.0) {
        const double range = std::hypot(point.x, point.y);
        if (range < min_range_ || range > max_range_) {
          continue;
        }
      }

      const int ix = static_cast<int>(std::floor(point.x / resolution_));
      const int iy = static_cast<int>(std::floor(point.y / resolution_));
      const std::int64_t key = make_key(ix, iy);
      auto cell = cell_counts_.find(key);
      if (cell == cell_counts_.end()) {
        if (cell_counts_.size() < static_cast<std::size_t>(max_cells_)) {
          cell_counts_.emplace(key, 1);
          update_bounds(ix, iy);
        }
      } else if (cell->second < std::numeric_limits<std::uint16_t>::max()) {
        cell->second += 1;
      }

      accepted_points += 1;
      if (accepted_points >= static_cast<std::size_t>(max_points_per_cloud_)) {
        break;
      }
    }

    has_cloud_ = true;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Cloud frame=%s raw=%zu finite=%zu accepted=%zu cells=%zu raw_z=[%.2f, %.2f]",
      latest_header_.frame_id.c_str(),
      point_count,
      finite_points,
      accepted_points,
      cell_counts_.size(),
      finite_points == 0 ? 0.0 : raw_min_z,
      finite_points == 0 ? 0.0 : raw_max_z
    );
  }

  void publish_map() {
    if (!has_cloud_) {
      return;
    }
    if (!has_cells_ || cell_counts_.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "No points left after filtering. Adjust min_z/max_z or cloud_topic."
      );
      return;
    }

    const int padding_cells = std::max(0, static_cast<int>(std::ceil(padding_ / resolution_)));
    const int origin_ix = min_ix_ - padding_cells;
    const int origin_iy = min_iy_ - padding_cells;
    const int width = std::max(1, max_ix_ - min_ix_ + 1 + 2 * padding_cells);
    const int height = std::max(1, max_iy_ - min_iy_ + 1 + 2 * padding_cells);

    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = this->now();
    map.header.frame_id = frame_id_.empty() ? latest_header_.frame_id : frame_id_;
    map.info.map_load_time = map.header.stamp;
    map.info.resolution = static_cast<float>(resolution_);
    map.info.width = static_cast<std::uint32_t>(width);
    map.info.height = static_cast<std::uint32_t>(height);
    map.info.origin.position.x = static_cast<double>(origin_ix) * resolution_;
    map.info.origin.position.y = static_cast<double>(origin_iy) * resolution_;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(static_cast<std::size_t>(width * height), -1);

    for (const auto & cell : cell_counts_) {
      if (cell.second < static_cast<std::uint16_t>(occupied_threshold_)) {
        continue;
      }
      const int ix = key_x(cell.first) - origin_ix;
      const int iy = key_y(cell.first) - origin_iy;
      if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
        map.data[static_cast<std::size_t>(iy * width + ix)] = 100;
      }
    }

    map_pub_->publish(map);
  }

  void update_bounds(int ix, int iy) {
    if (!has_cells_) {
      min_ix_ = max_ix_ = ix;
      min_iy_ = max_iy_ = iy;
      has_cells_ = true;
      return;
    }
    min_ix_ = std::min(min_ix_, ix);
    max_ix_ = std::max(max_ix_, ix);
    min_iy_ = std::min(min_iy_, iy);
    max_iy_ = std::max(max_iy_, iy);
  }

  static std::int64_t make_key(int ix, int iy) {
    return (static_cast<std::int64_t>(ix) << 32) |
      static_cast<std::uint32_t>(iy);
  }

  static int key_x(std::int64_t key) {
    return static_cast<int>(key >> 32);
  }

  static int key_y(std::int64_t key) {
    return static_cast<int>(static_cast<std::int32_t>(key & 0xffffffff));
  }

  static float read_float32(const std::vector<unsigned char> & data, std::size_t offset) {
    float value = 0.0f;
    std::memcpy(&value, data.data() + offset, sizeof(float));
    return value;
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2CloudMapProjector>());
  rclcpp::shutdown();
  return 0;
}
