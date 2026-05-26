#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

class Go2StateLogger : public rclcpp::Node {
public:
  Go2StateLogger() : Node("go2_state_logger") {
    output_path_ = this->declare_parameter<std::string>("output_path", "go2_state_log.csv");

    csv_.open(output_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error("Failed to open output CSV: " + output_path_);
    }

    csv_ << "time_s,"
         << "sport_x,sport_y,sport_z,sport_roll,sport_pitch,sport_yaw,"
         << "sport_vx,sport_vy,sport_vz,sport_yaw_speed,body_height,mode,gait_type,error_code,"
         << "odom_x,odom_y,odom_z,odom_roll,odom_pitch,odom_yaw,"
         << "odom_vx,odom_vy,odom_vz,odom_wx,odom_wy,odom_wz\n";
    csv_ << std::fixed << std::setprecision(6);

    sport_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "/sportmodestate",
      10,
      std::bind(&Go2StateLogger::on_sport_state, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/utlidar/robot_odom",
      10,
      std::bind(&Go2StateLogger::on_odom, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Logging Go2 state to %s", output_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing: /sportmodestate and /utlidar/robot_odom");
  }

  ~Go2StateLogger() override {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

private:
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::ofstream csv_;
  std::string output_path_;

  unitree_go::msg::SportModeState::SharedPtr latest_sport_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  template <typename Stamp>
  static double stamp_to_seconds(const Stamp & stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
  }

  static void quaternion_to_rpy(
    double x,
    double y,
    double z,
    double w,
    double & roll,
    double & pitch,
    double & yaw
  ) {
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
      pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
      pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  void on_sport_state(const unitree_go::msg::SportModeState::SharedPtr msg) {
    latest_sport_ = msg;
    write_row(stamp_to_seconds(msg->stamp));
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = msg;
  }

  void write_row(double time_s) {
    if (!latest_sport_) {
      return;
    }

    double odom_roll = 0.0;
    double odom_pitch = 0.0;
    double odom_yaw = 0.0;
    if (latest_odom_) {
      const auto & q = latest_odom_->pose.pose.orientation;
      quaternion_to_rpy(q.x, q.y, q.z, q.w, odom_roll, odom_pitch, odom_yaw);
    }

    const auto & sport = *latest_sport_;
    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_z = 0.0;
    double odom_vx = 0.0;
    double odom_vy = 0.0;
    double odom_vz = 0.0;
    double odom_wx = 0.0;
    double odom_wy = 0.0;
    double odom_wz = 0.0;
    if (latest_odom_) {
      odom_x = latest_odom_->pose.pose.position.x;
      odom_y = latest_odom_->pose.pose.position.y;
      odom_z = latest_odom_->pose.pose.position.z;
      odom_vx = latest_odom_->twist.twist.linear.x;
      odom_vy = latest_odom_->twist.twist.linear.y;
      odom_vz = latest_odom_->twist.twist.linear.z;
      odom_wx = latest_odom_->twist.twist.angular.x;
      odom_wy = latest_odom_->twist.twist.angular.y;
      odom_wz = latest_odom_->twist.twist.angular.z;
    }

    csv_ << time_s << ","
         << value_or_zero(sport.position, 0) << ","
         << value_or_zero(sport.position, 1) << ","
         << value_or_zero(sport.position, 2) << ","
         << value_or_zero(sport.imu_state.rpy, 0) << ","
         << value_or_zero(sport.imu_state.rpy, 1) << ","
         << value_or_zero(sport.imu_state.rpy, 2) << ","
         << value_or_zero(sport.velocity, 0) << ","
         << value_or_zero(sport.velocity, 1) << ","
         << value_or_zero(sport.velocity, 2) << ","
         << sport.yaw_speed << ","
         << sport.body_height << ","
         << static_cast<int>(sport.mode) << ","
         << static_cast<int>(sport.gait_type) << ","
         << sport.error_code << ","
         << odom_x << ","
         << odom_y << ","
         << odom_z << ","
         << odom_roll << ","
         << odom_pitch << ","
         << odom_yaw << ","
         << odom_vx << ","
         << odom_vy << ","
         << odom_vz << ","
         << odom_wx << ","
         << odom_wy << ","
         << odom_wz << "\n";

    if (static_cast<int64_t>(row_count_ % 50) == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "state x=%.3f y=%.3f yaw=%.3f vx=%.3f wz=%.3f",
        value_or_zero(sport.position, 0),
        value_or_zero(sport.position, 1),
        value_or_zero(sport.imu_state.rpy, 2),
        value_or_zero(sport.velocity, 0),
        sport.yaw_speed
      );
    }
    row_count_ += 1;
  }

  template <typename Sequence>
  static double value_or_zero(const Sequence & values, std::size_t index) {
    return index < values.size() ? static_cast<double>(values[index]) : 0.0;
  }

  std::size_t row_count_ = 0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2StateLogger>());
  rclcpp::shutdown();
  return 0;
}
