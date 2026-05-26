#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

#include <cstdint>
#include <vector>

using namespace std::chrono_literals;

namespace {

// Common Go2 Sport API ids. Verify against the installed
// unitree_api/msg/request_identity__struct.hpp if firmware/API versions differ.
constexpr int32_t kBalanceStandApiId = 1002;
constexpr int32_t kStopMoveApiId = 1003;
constexpr int32_t kHelloApiId = 1016;

struct SportStep {
  const char * name;
  int32_t api_id;
};

}  // namespace

class Go2SportTest : public rclcpp::Node {
public:
  Go2SportTest() : Node("go2_sport_test") {
    RCLCPP_INFO(this->get_logger(), "Go2 sport request sequence node started.");

    publisher_ = this->create_publisher<unitree_api::msg::Request>(
      "/api/sport/request",
      10
    );

    steps_ = {
      {"BalanceStand", kBalanceStandApiId},
      {"Hello", kHelloApiId},
      {"StopMove", kStopMoveApiId},
    };

    timer_ = this->create_wall_timer(
      2s,
      std::bind(&Go2SportTest::on_timer, this)
    );
  }

private:
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<SportStep> steps_;
  std::size_t step_index_ = 0;

  void send_sport_request(const SportStep & step) {
    unitree_api::msg::Request request;
    request.header.identity.api_id = step.api_id;

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing Sport API request: %s (api_id=%d)",
      step.name,
      step.api_id
    );

    publisher_->publish(request);
  }

  void on_timer() {
    if (step_index_ >= steps_.size()) {
      RCLCPP_INFO(this->get_logger(), "Sport request sequence finished.");
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    send_sport_request(steps_[step_index_]);
    step_index_ += 1;
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2SportTest>());
  rclcpp::shutdown();
  return 0;
}
