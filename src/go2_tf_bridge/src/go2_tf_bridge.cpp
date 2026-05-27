#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>
#include <string>

class Go2TfBridge : public rclcpp::Node {
public:
  Go2TfBridge() : Node("go2_tf_bridge") {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/utlidar/robot_odom");
    fallback_parent_frame_ = this->declare_parameter<std::string>("parent_frame", "odom");
    fallback_child_frame_ = this->declare_parameter<std::string>("child_frame", "base_link");
    planar_ = this->declare_parameter<bool>("planar", true);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Go2TfBridge::on_odom, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Bridging odometry topic %s to TF, fallback frames %s -> %s",
      odom_topic_.c_str(),
      fallback_parent_frame_.c_str(),
      fallback_child_frame_.c_str()
    );
  }

private:
  std::string odom_topic_;
  std::string fallback_parent_frame_;
  std::string fallback_child_frame_;
  bool planar_ = true;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id =
      msg->header.frame_id.empty() ? fallback_parent_frame_ : msg->header.frame_id;
    transform.child_frame_id =
      msg->child_frame_id.empty() ? fallback_child_frame_ : msg->child_frame_id;

    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    if (planar_) {
      transform.transform.translation.z = 0.0;

      const auto & orientation = msg->pose.pose.orientation;
      tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      );
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      tf2::Quaternion yaw_only;
      yaw_only.setRPY(0.0, 0.0, yaw);
      transform.transform.rotation.x = yaw_only.x();
      transform.transform.rotation.y = yaw_only.y();
      transform.transform.rotation.z = yaw_only.z();
      transform.transform.rotation.w = yaw_only.w();
    } else {
      transform.transform.translation.z = msg->pose.pose.position.z;
      transform.transform.rotation = msg->pose.pose.orientation;
    }

    tf_broadcaster_->sendTransform(transform);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2TfBridge>());
  rclcpp::shutdown();
  return 0;
}
