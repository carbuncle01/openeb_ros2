#ifndef OPENEB_ROS2__DRIVER_COMPONENT_HPP_
#define OPENEB_ROS2__DRIVER_COMPONENT_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <metavision/sdk/stream/camera.h>
#include <rclcpp/rclcpp.hpp>

namespace openeb_ros2
{

class DriverComponent : public rclcpp::Node
{
public:
  explicit DriverComponent(const rclcpp::NodeOptions & options);
  ~DriverComponent() override;

private:
  using EventPacket = event_camera_msgs::msg::EventPacket;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

  void open_camera();
  void stop_camera() noexcept;
  void on_raw_data(const std::uint8_t * data, std::size_t size);
  void publish_pending_packet(
    const std::chrono::steady_clock::time_point & publish_time);
  bool has_output_subscribers() const;
  void record_raw_callback(
    const std::chrono::steady_clock::time_point & callback_start,
    std::size_t bytes);
  void print_statistics();

  Metavision::Camera camera_;
  bool camera_open_{false};
  bool camera_running_{false};

  Metavision::CallbackId raw_callback_id_{0};
  bool raw_callback_active_{false};
  Metavision::CallbackId runtime_error_callback_id_{0};
  bool runtime_error_callback_active_{false};

  rclcpp::Publisher<EventPacket>::SharedPtr event_publisher_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  EventPacket::UniquePtr pending_packet_;

  std::string serial_;
  std::string encoding_;
  std::string frame_id_;
  std::int64_t packet_duration_us_{1000};
  std::size_t packet_size_bytes_{1000000};
  std::size_t reserve_size_{0};
  std::uint64_t sequence_{0};
  std::uint32_t width_{0};
  std::uint32_t height_{0};
  double statistics_interval_s_{1.0};
  bool debug_{false};

  bool has_published_{false};
  std::chrono::steady_clock::time_point last_publish_time_;
  std::chrono::steady_clock::time_point last_statistics_time_;

  std::atomic<std::uint64_t> raw_callbacks_{0};
  std::atomic<std::uint64_t> raw_bytes_{0};
  std::atomic<std::uint64_t> published_messages_{0};
  std::atomic<std::uint64_t> published_bytes_{0};
  std::atomic<std::uint64_t> no_subscriber_callbacks_{0};
  std::atomic<std::uint64_t> callback_time_ns_{0};
  std::atomic<std::uint64_t> callback_time_max_ns_{0};
  std::atomic<std::uint64_t> interarrival_time_ns_{0};
  std::atomic<std::uint64_t> interarrival_time_max_ns_{0};
  std::atomic<std::uint64_t> interarrival_samples_{0};
  std::atomic<std::int64_t> last_raw_arrival_ns_{0};
  std::atomic<std::uint64_t> pending_bytes_{0};
};

}  // namespace openeb_ros2

#endif  // OPENEB_ROS2__DRIVER_COMPONENT_HPP_
