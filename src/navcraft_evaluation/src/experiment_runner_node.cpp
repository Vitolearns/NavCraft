#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "navcraft_interfaces/msg/tracking_statistics.hpp"
#include "navcraft_interfaces/msg/tracking_error.hpp"

using namespace std::chrono_literals;

class ExperimentRunnerNode final : public rclcpp::Node {
 public:
  ExperimentRunnerNode() : Node("experiment_runner_node") {
    output_csv_ = declare_parameter<std::string>("output_csv", "experiment_results.csv");
    output_json_ = declare_parameter<std::string>("output_json", "");
    controller_name_ = declare_parameter<std::string>("controller_name", "unknown");
    scenario_name_ = declare_parameter<std::string>("scenario_name", "tracking_benchmark");
    path_file_ = declare_parameter<std::string>("path_file", "");
    csv_.open(output_csv_, std::ios::out | std::ios::trunc);
    if (!csv_) {
      throw std::runtime_error("failed to open experiment output: " + output_csv_);
    }
    csv_ << "stamp,controller,lateral_error,heading_error,target_speed,actual_speed,nearest_path_index\n";
    tracking_sub_ = create_subscription<navcraft_interfaces::msg::TrackingError>(
        "/tracking_error", 50,
        [this](navcraft_interfaces::msg::TrackingError::SharedPtr msg) { OnTrackingError(*msg); });
    stats_pub_ = create_publisher<navcraft_interfaces::msg::TrackingStatistics>(
        "/experiment/statistics", 10);
    timer_ = create_wall_timer(1s, [this]() { PublishStatistics(); });
  }

  ~ExperimentRunnerNode() override { WriteJsonReport(); }

 private:
  static std::string JsonEscape(const std::string& value) {
    std::ostringstream out;
    for (const char c : value) {
      switch (c) {
        case '\\':
          out << "\\\\";
          break;
        case '"':
          out << "\\\"";
          break;
        case '\n':
          out << "\\n";
          break;
        default:
          out << c;
          break;
      }
    }
    return out.str();
  }

  void OnTrackingError(const navcraft_interfaces::msg::TrackingError& error) {
    ++samples_;
    last_stamp_sec_ = rclcpp::Time(error.header.stamp).seconds();
    last_controller_name_ =
        error.controller_name.empty() ? controller_name_ : error.controller_name;
    last_nearest_path_index_ = error.nearest_path_index;
    sum_abs_lateral_error_ += std::abs(error.lateral_error);
    sum_abs_heading_error_ += std::abs(error.heading_error);
    csv_ << last_stamp_sec_ << ','
         << last_controller_name_ << ','
         << error.lateral_error << ','
         << error.heading_error << ','
         << error.target_speed << ','
         << error.actual_speed << ','
         << error.nearest_path_index << '\n';
  }

  void PublishStatistics() {
    navcraft_interfaces::msg::TrackingStatistics stats;
    stats.header.stamp = now();
    stats.controller_name = samples_ == 0 ? controller_name_ : last_controller_name_;
    stats.samples = samples_;
    stats.mean_abs_lateral_error_m = samples_ == 0 ? 0.0 : sum_abs_lateral_error_ / samples_;
    stats.mean_abs_heading_error_rad = samples_ == 0 ? 0.0 : sum_abs_heading_error_ / samples_;
    stats.last_nearest_path_index = last_nearest_path_index_;
    stats_pub_->publish(stats);
  }

  void WriteJsonReport() {
    if (output_json_.empty()) {
      return;
    }
    std::ofstream json(output_json_, std::ios::out | std::ios::trunc);
    if (!json) {
      RCLCPP_ERROR(get_logger(), "failed to open experiment json output: %s", output_json_.c_str());
      return;
    }
    const double mean_lateral = samples_ == 0 ? 0.0 : sum_abs_lateral_error_ / samples_;
    const double mean_heading = samples_ == 0 ? 0.0 : sum_abs_heading_error_ / samples_;
    json << "{\n"
         << "  \"scenario_name\": \"" << JsonEscape(scenario_name_) << "\",\n"
         << "  \"controller_name\": \"" << JsonEscape(last_controller_name_) << "\",\n"
         << "  \"path_file\": \"" << JsonEscape(path_file_) << "\",\n"
         << "  \"output_csv\": \"" << JsonEscape(output_csv_) << "\",\n"
         << "  \"samples\": " << samples_ << ",\n"
         << "  \"last_stamp_sec\": " << last_stamp_sec_ << ",\n"
         << "  \"mean_abs_lateral_error\": " << mean_lateral << ",\n"
         << "  \"mean_abs_heading_error\": " << mean_heading << ",\n"
         << "  \"sum_abs_lateral_error\": " << sum_abs_lateral_error_ << ",\n"
         << "  \"sum_abs_heading_error\": " << sum_abs_heading_error_ << ",\n"
         << "  \"last_nearest_path_index\": " << last_nearest_path_index_ << ",\n"
         << "  \"interface_chain\": \"reference_path -> tracking_controller -> tracking_cmd_vel -> cmd_vel_mux -> cmd_vel -> odom -> tracking_error\"\n"
         << "}\n";
  }

  std::string output_csv_;
  std::string output_json_;
  std::string controller_name_;
  std::string scenario_name_;
  std::string path_file_;
  std::ofstream csv_;
  size_t samples_ = 0;
  double last_stamp_sec_ = 0.0;
  double sum_abs_lateral_error_ = 0.0;
  double sum_abs_heading_error_ = 0.0;
  int last_nearest_path_index_ = -1;
  std::string last_controller_name_ = "unknown";
  rclcpp::Subscription<navcraft_interfaces::msg::TrackingError>::SharedPtr tracking_sub_;
  rclcpp::Publisher<navcraft_interfaces::msg::TrackingStatistics>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExperimentRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
