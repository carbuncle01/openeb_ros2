#ifndef OPENEB_ROS2__PREPROCESSOR_COMPONENT_HPP_
#define OPENEB_ROS2__PREPROCESSOR_COMPONENT_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/core/algorithms/periodic_frame_generation_algorithm.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace openeb_ros2
{

class PreprocessorComponent
  : public rclcpp::Node,
    public event_camera_codecs::EventProcessor
{
public:
  explicit PreprocessorComponent(const rclcpp::NodeOptions & options);

  void eventCD(
    std::uint64_t sensor_time, std::uint16_t x, std::uint16_t y,
    std::uint8_t polarity) override;
  bool eventExtTrigger(
    std::uint64_t sensor_time, std::uint8_t edge,
    std::uint8_t id) override;
  void finished() override;
  void rawData(const char * data, std::size_t size) override;

private:
  using EventPacket = event_camera_msgs::msg::EventPacket;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  using EventDecoderFactory =
    event_camera_codecs::DecoderFactory<EventPacket, PreprocessorComponent>;

  void on_packet(EventPacket::UniquePtr packet);
  bool preprocess(EventPacket & packet);
  void decode_for_event_image(const EventPacket & packet);
  void on_frame_generated(Metavision::timestamp ts_us, cv::Mat & frame);
  void start_or_update_frame_generator();
  void reset_event_image_decoder();
  bool has_output_subscribers() const;
  bool has_event_image_subscribers() const;
  void record_callback_time(
    const std::chrono::steady_clock::time_point & callback_start);
  void print_statistics();

  rclcpp::Subscription<EventPacket>::SharedPtr event_subscription_;
  rclcpp::Publisher<EventPacket>::SharedPtr event_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr event_image_publisher_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  std::unique_ptr<EventDecoderFactory> event_decoder_factory_;
  std::unique_ptr<Metavision::PeriodicFrameGenerationAlgorithm> frame_generation_algo_;
  std::vector<Metavision::EventCD> cd_buffer_;

  std::string expected_encoding_;
  std::string output_frame_id_;
  std::string event_image_encoding_;
  std::string event_image_frame_id_;
  bool drop_empty_packets_{true};
  bool drop_unexpected_encoding_{true};
  bool event_image_enabled_{true};
  bool event_image_publish_empty_{true};
  bool event_image_subscriber_active_{false};
  bool debug_{false};
  double event_image_fps_{25.0};
  double statistics_interval_s_{1.0};
  std::uint32_t event_image_width_{0};
  std::uint32_t event_image_height_{0};
  std::uint32_t event_image_channels_{3};
  std::uint8_t event_image_background_value_{0};
  std::uint64_t decoded_events_in_packet_{0};
  std::uint64_t out_of_bounds_events_in_packet_{0};
  std::uint64_t events_in_active_image_{0};
  std::chrono::steady_clock::time_point last_statistics_time_;

  std::atomic<std::uint64_t> received_messages_{0};
  std::atomic<std::uint64_t> received_bytes_{0};
  std::atomic<std::uint64_t> published_messages_{0};
  std::atomic<std::uint64_t> published_bytes_{0};
  std::atomic<std::uint64_t> dropped_empty_{0};
  std::atomic<std::uint64_t> dropped_encoding_{0};
  std::atomic<std::uint64_t> no_subscriber_messages_{0};
  std::atomic<std::uint64_t> callback_time_ns_{0};
  std::atomic<std::uint64_t> callback_time_max_ns_{0};
  std::atomic<std::uint64_t> transport_latency_ns_{0};
  std::atomic<std::uint64_t> transport_latency_max_ns_{0};
  std::atomic<std::uint64_t> transport_latency_samples_{0};
  std::atomic<std::uint64_t> decoded_events_{0};
  std::atomic<std::uint64_t> decode_calls_{0};
  std::atomic<std::uint64_t> decode_time_ns_{0};
  std::atomic<std::uint64_t> decode_time_max_ns_{0};
  std::atomic<std::uint64_t> decode_errors_{0};
  std::atomic<std::uint64_t> out_of_bounds_events_{0};
  std::atomic<std::uint64_t> image_published_messages_{0};
  std::atomic<std::uint64_t> image_published_bytes_{0};
  std::atomic<std::uint64_t> image_published_events_{0};
  std::atomic<std::uint64_t> image_timer_calls_{0};
  std::atomic<std::uint64_t> image_timer_time_ns_{0};
  std::atomic<std::uint64_t> image_timer_time_max_ns_{0};
  std::atomic<std::uint64_t> image_no_subscriber_packets_{0};
};

}  // namespace openeb_ros2

#endif  // OPENEB_ROS2__PREPROCESSOR_COMPONENT_HPP_
