#ifndef OPENEB_ROS2__TENSOR_TIMING_HPP_
#define OPENEB_ROS2__TENSOR_TIMING_HPP_
#include <chrono>
#include <sstream>
#include <string>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
namespace openeb_ros2 {
inline std::string timing_quote(const std::string & value) {
  std::ostringstream out; out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {out << '\\' << static_cast<char>(c);}
    else if (c < 32) {const char *hex="0123456789abcdef"; out << "\\u00" << hex[c >> 4] << hex[c & 15];}
    else {out << static_cast<char>(c);}
  }
  out << '"'; return out.str();
}
class TensorTiming {
public:
  explicit TensorTiming(rclcpp::Node & node) : node_(node) {
    stream_id_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    publisher_ = node.create_publisher<std_msgs::msg::String>("tensor_timing", rclcpp::QoS(512).reliable());
  }
  void reset_epoch() {++epoch_;}
  void publish(const std_msgs::msg::Header & header, std::int64_t start_us,
    std::int64_t end_us, std::int64_t offset_ns, std::int64_t latest_us,
    const std::string & name, std::int64_t channels, std::int64_t height, std::int64_t width) {
    const auto sequence = sequence_++;
    if (publisher_->get_subscription_count() == 0) {return;}
    const auto stamp = static_cast<std::int64_t>(header.stamp.sec)*1000000000LL + header.stamp.nanosec;
    std::ostringstream out;
    out << "{\"schema\":\"openeb.tensor_timing.v1\",\"stream_id\":" << timing_quote(stream_id_)
        << ",\"sequence\":" << sequence << ",\"sensor_epoch\":" << epoch_
        << ",\"tensor_stamp_ros_ns\":" << stamp
        << ",\"generated_ros_ns\":" << node_.now().nanoseconds()
        << ",\"window_start_sensor_us\":" << start_us << ",\"window_end_sensor_us\":" << end_us
        << ",\"sensor_to_ros_offset_ns\":" << offset_ns
        << ",\"latest_event_sensor_us\":" << latest_us
        << ",\"window_start_ros_ns\":" << start_us*1000LL+offset_ns
        << ",\"window_end_ros_ns\":" << end_us*1000LL+offset_ns
        << ",\"tensor_name\":" << timing_quote(name) << ",\"frame_id\":" << timing_quote(header.frame_id)
        << ",\"shape\":[1," << channels << ',' << height << ',' << width
        << "],\"offset_method\":\"packet_receipt_minus_latest_event\",\"hardware_synchronized\":false}";
    std_msgs::msg::String message; message.data=out.str(); publisher_->publish(message);
  }
private:
  rclcpp::Node & node_;
  std::uint64_t sequence_{0}, epoch_{0};
  std::string stream_id_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
}
#endif
