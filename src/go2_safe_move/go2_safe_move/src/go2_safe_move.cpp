#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

#include <cstdint>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

// Official Unitree Go2 Sport API ids used by unitree_ros2 examples.
constexpr int32_t kBalanceStandApiId = 1002;
constexpr int32_t kStopMoveApiId = 1003;
constexpr int32_t kMoveApiId = 1008;

struct SafeMoveStep {
  const char * name;
  int32_t api_id;
  std::string parameter;
};

std::string move_parameter(double vx, double vy, double vyaw) {
  return "{"
    "\"x\":" + std::to_string(vx) + ","
    "\"y\":" + std::to_string(vy) + ","
    "\"z\":" + std::to_string(vyaw) +
  "}";
}

}  // namespace

class Go2SafeMove : public rclcpp::Node {
public:
  Go2SafeMove() : Node("go2_safe_move") {
    RCLCPP_INFO(this->get_logger(), "Go2 safe low-speed movement test started.");
    RCLCPP_INFO(this->get_logger(), "Keep the remote controller ready to take over.");

    publisher_ = this->create_publisher<unitree_api::msg::Request>(
      "/api/sport/request",
      10
    );

    steps_ = {
      {"BalanceStand", kBalanceStandApiId, ""},
      {"Move forward slowly", kMoveApiId, move_parameter(0.3, 0.0, 0.0)},
      {"StopMove", kStopMoveApiId, ""},
      {"Turn slowly", kMoveApiId, move_parameter(0.0, 0.0, 0.50)},
      {"StopMove", kStopMoveApiId, ""},
    };

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&Go2SafeMove::on_timer, this)
    );
  }

private:
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<SafeMoveStep> steps_;
  std::size_t tick_count_ = 0;

  void publish_request(const SafeMoveStep & step) {
    unitree_api::msg::Request request;
    request.header.identity.api_id = step.api_id;
    request.parameter = step.parameter;

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing %s: api_id=%d parameter=%s",
      step.name,
      step.api_id,
      step.parameter.empty() ? "{}" : step.parameter.c_str()
    );

    publisher_->publish(request);
  }

  void on_timer() {
    // Timeline:
    // 1s: BalanceStand
    // 3s: Move forward slowly
    // 4s: StopMove
    // 6s: Turn slowly
    // 7s: StopMove
    // 8s: shutdown
    if (tick_count_ == 0) {
      publish_request(steps_[0]);
    } else if (tick_count_ == 2) {
      publish_request(steps_[1]);
    } else if (tick_count_ == 3) {
      publish_request(steps_[2]);
    } else if (tick_count_ == 5) {
      publish_request(steps_[3]);
    } else if (tick_count_ == 6) {
      publish_request(steps_[4]);
    } else if (tick_count_ >= 7) {
      RCLCPP_INFO(this->get_logger(), "Safe movement test finished.");
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    tick_count_ += 1;
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2SafeMove>());
  rclcpp::shutdown();
  return 0;
}
