#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "navcraft_interfaces/action/navigate_sequence.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using Sequence = navcraft_interfaces::action::NavigateSequence;
using Mission = rclcpp_action::ServerGoalHandle<Sequence>;
using Nav = nav2_msgs::action::NavigateToPose;
using NavHandle = rclcpp_action::ClientGoalHandle<Nav>;
using Clock = std::chrono::steady_clock;

// All callbacks run in one mutually-exclusive group on a single-threaded executor.
// No detached workers, blocking futures or second navigation backend.
class SequenceExecutor final : public rclcpp::Node {
 public:
  SequenceExecutor() : Node("navigate_sequence_server_node") {
    server_timeout_ = positive("server_timeout_s", 10.0);
    response_timeout_ = positive("response_timeout_s", 5.0);
    goal_timeout_ = positive("goal_timeout_s", 120.0);
    cancel_timeout_ = positive("cancel_timeout_s", 5.0);
    client_ = rclcpp_action::create_client<Nav>(this, "navigate_to_pose");
    state_pub_ = create_publisher<std_msgs::msg::String>("execution_state", 10);
    server_ = rclcpp_action::create_server<Sequence>(
        this, "navigate_sequence",
        [this](const auto&, const std::shared_ptr<const Sequence::Goal> g) {
          if (state_ != "IDLE" || g->loop || g->goals.empty() || g->goals.size() > 100)
            return rclcpp_action::GoalResponse::REJECT;
          for (const auto& p : g->goals) {
            const auto& q = p.pose.orientation;
            const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
            if (p.header.frame_id != "map" || !std::isfinite(p.pose.position.x) ||
                !std::isfinite(p.pose.position.y) || !std::isfinite(p.pose.position.z) ||
                !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01)
              return rclcpp_action::GoalResponse::REJECT;
          }
          set_state("RESERVED");  // Reserve before the accepted callback.
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<Mission> h) {
          if (h != mission_ || state_ == "FAULT") return rclcpp_action::CancelResponse::REJECT;
          stop("cancel");
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<Mission> h) {
          mission_ = h;
          index_ = retries_ = 0;
          reason_.clear();
          nav_.reset();
          pending_ = false;
          wait_server();
        });
    pause_ = create_service<std_srvs::srv::Trigger>("pause_mission",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
          r->success = mission_ && (state_ == "RUNNING" || state_ == "SENDING" ||
                                   state_ == "WAIT_SERVER" || state_ == "PAUSED" || state_ == "PAUSING");
          if (r->success && state_ != "PAUSED" && state_ != "PAUSING") stop("pause");
          r->message = state_;
        });
    resume_ = create_service<std_srvs::srv::Trigger>("resume_mission",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
          r->success = mission_ && state_ == "PAUSED";
          if (r->success) { reason_.clear(); wait_server(); }
          r->message = state_;
        });
    cancel_ = create_service<std_srvs::srv::Trigger>("cancel_mission",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
          r->success = mission_ && state_ != "FAULT";
          if (r->success) stop("cancel");
          r->message = state_;
        });
    timer_ = create_wall_timer(50ms, [this]() { tick(); });
  }

 private:
  double positive(const std::string& name, double value) {
    value = declare_parameter<double>(name, value);
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name);
    return value;
  }
  void deadline(double seconds) {
    deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
  }
  void set_state(const std::string& s) {
    state_ = s;
    std_msgs::msg::String message;
    message.data = s;
    state_pub_->publish(message);
  }
  void wait_server() { set_state("WAIT_SERVER"); deadline(server_timeout_); }
  void finish(bool success, const std::string& message, bool fault = false) {
    if (mission_) {
      auto r = std::make_shared<Sequence::Result>();
      r->success = success;
      r->message = message;
      r->succeeded_goals = static_cast<int>(index_);
      r->failed_goals = success || reason_ == "cancel" ? 0 : 1;
      if (mission_->is_canceling()) mission_->canceled(r);
      else if (success) mission_->succeed(r);
      else mission_->abort(r);
      mission_.reset();
    }
    set_state(fault ? "FAULT" : "IDLE");
    // In FAULT retain ownership of an outstanding request; a late acceptance
    // is still cancelled. Never admit another mission with unknown Nav2 state.
  }
  void cancel_nav() {
    if (!nav_ || cancel_sent_) return;
    cancel_sent_ = true;
    try { client_->async_cancel_goal(nav_); }
    catch (const std::exception& e) { finish(false, e.what(), true); }
  }
  void stop(const std::string& reason) {
    if (state_ == "CANCELING" || state_ == "FAULT") return;
    const bool already_stopping = state_ == "PAUSING";
    reason_ = reason;
    set_state(reason == "pause" ? "PAUSING" : "CANCELING");
    if (!already_stopping) deadline(cancel_timeout_);
    cancel_nav();
    // Finish on the next tick so the outer action's cancel state is committed.
  }
  void stopped() {
    if (reason_ == "pause") set_state("PAUSED");
    else finish(false, reason_ == "cancel" ? "cancelled" : "navigation deadline exceeded");
  }
  void send() {
    if (index_ == mission_->get_goal()->goals.size()) { finish(true, "sequence completed"); return; }
    Nav::Goal g;
    g.pose = mission_->get_goal()->goals[index_];
    g.pose.header.stamp = now();
    const auto token = ++generation_;
    pending_ = true;
    cancel_sent_ = false;
    set_state("SENDING");
    deadline(response_timeout_);
    rclcpp_action::Client<Nav>::SendGoalOptions options;
    options.goal_response_callback = [this, token](NavHandle::SharedPtr h) {
      if (token != generation_) { if (h) client_->async_cancel_goal(h); return; }
      pending_ = false;
      nav_ = h;
      if (state_ == "FAULT") { cancel_nav(); return; }
      if (!h) {
        if (state_ == "PAUSING" || state_ == "CANCELING") stopped();
        else finish(false, "Nav2 rejected goal");
      } else if (state_ == "PAUSING" || state_ == "CANCELING") cancel_nav();
      else { set_state("RUNNING"); deadline(goal_timeout_); }
    };
    options.result_callback = [this, token](const NavHandle::WrappedResult& r) {
      if (token != generation_) return;
      nav_.reset();
      pending_ = false;
      if (state_ == "FAULT" || !mission_) return;
      const bool success = r.code == rclcpp_action::ResultCode::SUCCEEDED;
      if (success) { ++index_; retries_ = 0; }
      if (state_ == "PAUSING" || state_ == "CANCELING") { stopped(); return; }
      if (success) wait_server();
      else if (r.code == rclcpp_action::ResultCode::ABORTED && retries_++ == 0) {
        RCLCPP_WARN(get_logger(), "Nav2 aborted waypoint; retrying once");
        wait_server();
      } else finish(false, "Nav2 failed; retry budget exhausted or unexpected cancellation");
    };
    try { client_->async_send_goal(g, options); }
    catch (const std::exception& e) { finish(false, e.what(), true); }
  }
  void tick() {
    set_state(state_);  // Heartbeat: the velocity gate fails closed if this stops.
    if (!mission_) return;
    auto f = std::make_shared<Sequence::Feedback>();
    f->current_goal_index = static_cast<int>(index_);
    f->total_goals = static_cast<int>(mission_->get_goal()->goals.size());
    f->state = state_;
    f->message = reason_;
    mission_->publish_feedback(f);
    if (state_ == "WAIT_SERVER") {
      if (index_ == mission_->get_goal()->goals.size()) finish(true, "sequence completed");
      else if (client_->action_server_is_ready()) send();
      else if (Clock::now() >= deadline_) finish(false, "Nav2 action server unavailable");
    } else if (state_ == "SENDING" && Clock::now() >= deadline_) {
      finish(false, "Nav2 acceptance timeout; outcome unknown, restart after Nav2 is stopped", true);
    } else if (state_ == "RUNNING" && Clock::now() >= deadline_) stop("timeout");
    else if (state_ == "PAUSING" || state_ == "CANCELING") {
      if (!pending_ && !nav_) stopped();
      else if (Clock::now() >= deadline_) finish(false, "Nav2 cancellation timeout; restart after Nav2 is stopped", true);
    }
  }
  std::string state_{"IDLE"}, reason_;
  size_t index_{0};
  unsigned retries_{0};
  uint64_t generation_{0};
  bool pending_{false}, cancel_sent_{false};
  double server_timeout_, response_timeout_, goal_timeout_, cancel_timeout_;
  Clock::time_point deadline_;
  std::shared_ptr<Mission> mission_;
  NavHandle::SharedPtr nav_;
  rclcpp_action::Client<Nav>::SharedPtr client_;
  rclcpp_action::Server<Sequence>::SharedPtr server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_, resume_, cancel_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceExecutor>());
  rclcpp::shutdown();
  return 0;
}
