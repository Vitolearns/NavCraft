#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class VelocityGate final : public rclcpp::Node {
 public:
  VelocityGate() : Node("cmd_vel_safety_gate_node") {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    command_ = create_subscription<geometry_msgs::msg::Twist>("nav2_cmd_vel", 1,
        [this](const geometry_msgs::msg::Twist& m) { last_ = m; command_at_ = Clock::now(); });
    state_ = create_subscription<std_msgs::msg::String>("execution_state", 1,
        [this](const std_msgs::msg::String& m) {
          running_ = m.data == "RUNNING";
          state_at_ = Clock::now();
          if (!running_) { last_ = geometry_msgs::msg::Twist(); command_at_ = {}; publish(); }
        });
    estop_ = create_service<std_srvs::srv::SetBool>("emergency_stop",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> q,
               std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
          stopped_ = q->data;
          last_ = geometry_msgs::msg::Twist(); command_at_ = {};
          publish();
          r->success = true; r->message = stopped_ ? "latched" : "released; awaiting fresh velocity";
        });
    timer_ = create_wall_timer(20ms, [this]() { publish(); });
  }
 private:
  void publish() {
    geometry_msgs::msg::Twist out;
    const auto now = Clock::now();
    if (running_ && !stopped_ && now - state_at_ < 300ms && now - command_at_ < 300ms &&
        std::isfinite(last_.linear.x) && std::isfinite(last_.angular.z)) {
      out.linear.x = std::clamp(last_.linear.x, -0.5, 0.5);
      out.angular.z = std::clamp(last_.angular.z, -1.2, 1.2);
    }
    pub_->publish(out);
  }
  bool running_{false}, stopped_{false};
  Clock::time_point state_at_{}, command_at_{};
  geometry_msgs::msg::Twist last_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityGate>());
  rclcpp::shutdown();
  return 0;
}
