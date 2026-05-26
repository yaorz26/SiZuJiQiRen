#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

using namespace std::chrono_literals;

class HelloWorld: public rclcpp::Node {
public:
  HelloWorld() : Node("hello_world") {
    RCLCPP_INFO(this->get_logger(), "Hello, ROS2!");
    // 创建发布对象
    publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    // 创建定时器，每秒调用一次回调函数
    timer_ = this->create_wall_timer(
      1s,
      std::bind(&HelloWorld::on_timer, this)
    );
  }
private:
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  // 定时器回调函数
  void on_timer() {
    unitree_api::msg::Request request;

    // 打招呼对应的api_id是1016，具体可以参考unitree_api/msg/request_identity__struct.hpp中的定义
    request.header.identity.api_id = 1016;

    publisher_ -> publish(request);
  }
};

int main(int argc, char * argv[]) {
  // 初始化ros2客户端
  rclcpp::init(argc, argv);
  // 调用spin函数，并传入节点对象指针
  rclcpp::spin(std::make_shared<HelloWorld>());
  // 释放资源，关闭ros2客户端
  rclcpp::shutdown();
  return 0;
}