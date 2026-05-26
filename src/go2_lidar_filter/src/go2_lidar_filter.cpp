#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

struct PointXYZ {
  double x;
  double y;
  double z;
};

struct GridSpec {
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  double resolution = 0.10;
  int width = 0;
  int height = 0;
};

class Go2LidarFilter : public rclcpp::Node {
public:
  Go2LidarFilter() : Node("go2_lidar_filter") {
    input_csv_ = this->declare_parameter<std::string>("input_csv", "go2_dense_points.csv");
    output_prefix_ = this->declare_parameter<std::string>("output_prefix", "go2_filtered");
    max_range_m_ = this->declare_parameter<double>("max_range_m", 8.0);
    min_z_m_ = this->declare_parameter<double>("min_z_m", -1.0);
    max_z_m_ = this->declare_parameter<double>("max_z_m", 2.0);
    ground_max_z_m_ = this->declare_parameter<double>("ground_max_z_m", 0.12);
    obstacle_min_z_m_ = this->declare_parameter<double>("obstacle_min_z_m", 0.15);
    obstacle_max_z_m_ = this->declare_parameter<double>("obstacle_max_z_m", 1.60);
    voxel_size_m_ = this->declare_parameter<double>("voxel_size_m", 0.03);
    grid_resolution_m_ = this->declare_parameter<double>("grid_resolution_m", 0.10);
    grid_padding_m_ = this->declare_parameter<double>("grid_padding_m", 0.50);

    if (voxel_size_m_ <= 0.0 || grid_resolution_m_ <= 0.0) {
      throw std::runtime_error("voxel_size_m and grid_resolution_m must be positive");
    }

    run();
    rclcpp::shutdown();
  }

private:
  std::string input_csv_;
  std::string output_prefix_;
  double max_range_m_ = 8.0;
  double min_z_m_ = -1.0;
  double max_z_m_ = 2.0;
  double ground_max_z_m_ = 0.12;
  double obstacle_min_z_m_ = 0.15;
  double obstacle_max_z_m_ = 1.60;
  double voxel_size_m_ = 0.03;
  double grid_resolution_m_ = 0.10;
  double grid_padding_m_ = 0.50;

  void run() {
    const auto raw_points = read_csv(input_csv_);
    RCLCPP_INFO(this->get_logger(), "Loaded %zu raw points from %s", raw_points.size(), input_csv_.c_str());

    std::vector<PointXYZ> filtered;
    std::vector<PointXYZ> ground;
    std::vector<PointXYZ> obstacles;
    filtered.reserve(raw_points.size());

    for (const auto & point : raw_points) {
      const double range_xy = std::hypot(point.x, point.y);
      if (range_xy > max_range_m_ || point.z < min_z_m_ || point.z > max_z_m_) {
        continue;
      }
      filtered.push_back(point);
      if (point.z <= ground_max_z_m_) {
        ground.push_back(point);
      }
      if (point.z >= obstacle_min_z_m_ && point.z <= obstacle_max_z_m_) {
        obstacles.push_back(point);
      }
    }

    auto filtered_ds = voxel_downsample(filtered, voxel_size_m_);
    auto ground_ds = voxel_downsample(ground, voxel_size_m_);
    auto obstacles_ds = voxel_downsample(obstacles, voxel_size_m_);

    write_ply(output_prefix_ + "_filtered.ply", filtered_ds);
    write_ply(output_prefix_ + "_ground.ply", ground_ds);
    write_ply(output_prefix_ + "_obstacles.ply", obstacles_ds);
    write_csv(output_prefix_ + "_obstacles.csv", obstacles_ds);
    write_occupancy_grid(output_prefix_, obstacles_ds);

    RCLCPP_INFO(
      this->get_logger(),
      "Filtered=%zu Ground=%zu Obstacles=%zu after voxel downsample",
      filtered_ds.size(),
      ground_ds.size(),
      obstacles_ds.size()
    );
    RCLCPP_INFO(
      this->get_logger(),
      "Outputs: %s_filtered.ply, %s_ground.ply, %s_obstacles.ply, %s_occupancy.csv, %s_occupancy.pgm",
      output_prefix_.c_str(),
      output_prefix_.c_str(),
      output_prefix_.c_str(),
      output_prefix_.c_str(),
      output_prefix_.c_str()
    );
  }

  static std::vector<PointXYZ> read_csv(const std::string & path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open input CSV: " + path);
    }

    std::vector<PointXYZ> points;
    std::string line;
    bool first_line = true;
    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }
      if (first_line) {
        first_line = false;
        if (line.find("x") != std::string::npos && line.find("y") != std::string::npos) {
          continue;
        }
      }

      std::stringstream ss(line);
      std::string xs;
      std::string ys;
      std::string zs;
      if (!std::getline(ss, xs, ',') || !std::getline(ss, ys, ',') || !std::getline(ss, zs, ',')) {
        continue;
      }
      PointXYZ point{std::stod(xs), std::stod(ys), std::stod(zs)};
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        points.push_back(point);
      }
    }
    return points;
  }

  static std::vector<PointXYZ> voxel_downsample(const std::vector<PointXYZ> & points, double voxel_size) {
    std::vector<PointXYZ> result;
    result.reserve(points.size());
    std::unordered_set<std::int64_t> seen;
    seen.reserve(points.size());

    for (const auto & point : points) {
      const auto ix = static_cast<std::int64_t>(std::floor(point.x / voxel_size));
      const auto iy = static_cast<std::int64_t>(std::floor(point.y / voxel_size));
      const auto iz = static_cast<std::int64_t>(std::floor(point.z / voxel_size));
      const std::int64_t key = hash_voxel(ix, iy, iz);
      if (seen.insert(key).second) {
        result.push_back(point);
      }
    }
    return result;
  }

  static std::int64_t hash_voxel(std::int64_t x, std::int64_t y, std::int64_t z) {
    const std::int64_t p1 = 73856093;
    const std::int64_t p2 = 19349663;
    const std::int64_t p3 = 83492791;
    return (x * p1) ^ (y * p2) ^ (z * p3);
  }

  static void write_ply(const std::string & path, const std::vector<PointXYZ> & points) {
    std::ofstream file(path, std::ios::out | std::ios::trunc);
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << points.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
    for (const auto & point : points) {
      file << point.x << " " << point.y << " " << point.z << "\n";
    }
  }

  static void write_csv(const std::string & path, const std::vector<PointXYZ> & points) {
    std::ofstream file(path, std::ios::out | std::ios::trunc);
    file << "x,y,z\n";
    for (const auto & point : points) {
      file << point.x << "," << point.y << "," << point.z << "\n";
    }
  }

  void write_occupancy_grid(const std::string & prefix, const std::vector<PointXYZ> & obstacles) const {
    if (obstacles.empty()) {
      std::ofstream csv(prefix + "_occupancy.csv", std::ios::out | std::ios::trunc);
      csv << "ix,iy,x,y,occupied\n";
      return;
    }

    GridSpec spec = make_grid_spec(obstacles);
    std::vector<std::uint8_t> grid(static_cast<std::size_t>(spec.width * spec.height), 0);

    for (const auto & point : obstacles) {
      const int ix = static_cast<int>(std::floor((point.x - spec.x_min) / spec.resolution));
      const int iy = static_cast<int>(std::floor((point.y - spec.y_min) / spec.resolution));
      if (ix >= 0 && ix < spec.width && iy >= 0 && iy < spec.height) {
        grid[static_cast<std::size_t>(iy * spec.width + ix)] = 1;
      }
    }

    std::ofstream csv(prefix + "_occupancy.csv", std::ios::out | std::ios::trunc);
    csv << "ix,iy,x,y,occupied\n";
    for (int iy = 0; iy < spec.height; ++iy) {
      for (int ix = 0; ix < spec.width; ++ix) {
        const auto occupied = grid[static_cast<std::size_t>(iy * spec.width + ix)];
        if (!occupied) {
          continue;
        }
        const double x = spec.x_min + (static_cast<double>(ix) + 0.5) * spec.resolution;
        const double y = spec.y_min + (static_cast<double>(iy) + 0.5) * spec.resolution;
        csv << ix << "," << iy << "," << x << "," << y << ",1\n";
      }
    }

    std::ofstream pgm(prefix + "_occupancy.pgm", std::ios::out | std::ios::trunc);
    pgm << "P2\n";
    pgm << spec.width << " " << spec.height << "\n";
    pgm << "255\n";
    for (int iy = spec.height - 1; iy >= 0; --iy) {
      for (int ix = 0; ix < spec.width; ++ix) {
        const auto occupied = grid[static_cast<std::size_t>(iy * spec.width + ix)];
        pgm << (occupied ? 0 : 255) << " ";
      }
      pgm << "\n";
    }

    std::ofstream meta(prefix + "_occupancy_meta.txt", std::ios::out | std::ios::trunc);
    meta << "x_min " << spec.x_min << "\n";
    meta << "x_max " << spec.x_max << "\n";
    meta << "y_min " << spec.y_min << "\n";
    meta << "y_max " << spec.y_max << "\n";
    meta << "resolution " << spec.resolution << "\n";
    meta << "width " << spec.width << "\n";
    meta << "height " << spec.height << "\n";
  }

  GridSpec make_grid_spec(const std::vector<PointXYZ> & points) const {
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();
    for (const auto & point : points) {
      x_min = std::min(x_min, point.x);
      x_max = std::max(x_max, point.x);
      y_min = std::min(y_min, point.y);
      y_max = std::max(y_max, point.y);
    }

    x_min -= grid_padding_m_;
    x_max += grid_padding_m_;
    y_min -= grid_padding_m_;
    y_max += grid_padding_m_;

    GridSpec spec;
    spec.x_min = x_min;
    spec.x_max = x_max;
    spec.y_min = y_min;
    spec.y_max = y_max;
    spec.resolution = grid_resolution_m_;
    spec.width = std::max(1, static_cast<int>(std::ceil((x_max - x_min) / grid_resolution_m_)));
    spec.height = std::max(1, static_cast<int>(std::ceil((y_max - y_min) / grid_resolution_m_)));
    return spec;
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2LidarFilter>());
  rclcpp::shutdown();
  return 0;
}
