#ifndef OPENEB_ROS2__EVENT_TENSOR_ENCODER_NODE_HPP_
#define OPENEB_ROS2__EVENT_TENSOR_ENCODER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <utility>

#include "cuda_runtime_api.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "event_camera_codecs/decoder_factory.h"
#include "event_camera_codecs/event_processor.h"
#include "event_camera_msgs/msg/event_packet.hpp"
#include "openeb_ros2/event_tensor_cuda_backend.hpp"
#include "isaac_ros_common/cuda_stream.hpp"
#include "openeb_ros2/tensor_output.hpp"
#include "isaac_ros_tensor_msgs/msg/tensor_list.hpp"
#include "metavision/sdk/base/events/event_cd.h"
#include "rclcpp/rclcpp.hpp"

namespace openeb_ros2
{

class EventTensorEncoderNode
  : public rclcpp::Node,
    public event_camera_codecs::EventProcessor
{
public:
  explicit EventTensorEncoderNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~EventTensorEncoderNode() override;

  void eventCD(
    std::uint64_t sensor_time, std::uint16_t x, std::uint16_t y,
    std::uint8_t polarity) override;
  bool eventExtTrigger(
    std::uint64_t sensor_time, std::uint8_t edge,
    std::uint8_t id) override;
  void finished() override;
  void rawData(const char * data, std::size_t size) override;

  // Direct input transfers ownership; caller must not retain the SDK buffer.
  void ingest(event_camera_msgs::msg::EventPacket::UniquePtr packet) { on_packet(std::move(packet)); }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr configuration_guard_;
  std::recursive_mutex processing_mutex_;
  using EventPacket = event_camera_msgs::msg::EventPacket;
  using TensorList = isaac_ros_tensor_msgs::msg::TensorList;
  using EventDecoderFactory =
    event_camera_codecs::DecoderFactory<EventPacket, EventTensorEncoderNode>;
  using Timestamp = Metavision::timestamp;

  void on_packet(EventPacket::UniquePtr packet);
  void process_events(const std::vector<Metavision::EventCD> & events);
  void flush_cuda_events();
  void maybe_publish_cuda_snapshot(Timestamp window_end_us);
  void on_inference_output(TensorList::ConstSharedPtr message);
  void on_cuda_timer();
  void publish_window(Timestamp window_end_us, const std::string & frame_id);
  void build_full_histogram(Timestamp window_start_us, Timestamp window_end_us);
  void build_incremental_histogram(Timestamp window_start_us, Timestamp window_end_us);
  void accumulate_histogram_event(
    const Metavision::EventCD & event, Timestamp window_start_us,
    Timestamp window_end_us, std::vector<float> & tensor) const;
  void accumulate_linear_event(
    const Metavision::EventCD & event, Timestamp window_start_us,
    Timestamp window_end_us, std::vector<float> & tensor) const;
  std::size_t channel_index(std::size_t bin, bool positive) const;
  void prepare_staging_buffer(std::vector<float> & output) const;
  bool accept_event_timestamp(Timestamp event_us);
  void reset_state(const std::string & reason);
  void publish_diagnostics();
  void initialize_publish_schedule(Timestamp first_event_us);
  void advance_publish_schedule();
  bool incremental_eligible() const;
  std::size_t incremental_shift_bins() const;
  std::int64_t ros_timestamp_ns(Timestamp sensor_timestamp_us) const;

  std::int64_t bins_{10};
  std::int64_t width_{0};
  std::int64_t height_{0};
  std::int64_t window_us_{40000};
  std::int64_t stride_us_{4000};
  std::int64_t publish_period_us_{4000};
  double output_rate_hz_{0.0};
  std::string polarity_mode_{"separate"};
  std::string polarity_layout_{"polarity_major"};
  std::string temporal_interpolation_{"none"};
  std::string incremental_mode_{"auto"};
  std::string representation_backend_{"cpu"};
  std::string inference_policy_{"periodic"};
  std::string tensor_name_{"input_tensor"};
  std::vector<double> channel_mean_;
  std::vector<double> channel_stddev_;
  bool publish_empty_{true};
  bool use_pinned_host_memory_{true};
  bool debug_{false};
  double statistics_interval_s_{1.0};
  std::int64_t cuda_update_us_{1000};
  std::int64_t timestamp_backward_tolerance_us_{4000};
  std::size_t cuda_events_per_transfer_{8192};
  double inference_watchdog_ms_{100.0};
  std::size_t channels_{20};
  std::size_t pixels_{0};
  std::size_t tensor_elements_{0};
  std::size_t tensor_bytes_{0};
  bool use_incremental_{false};
  bool normalization_identity_{true};
  std::size_t shift_bins_{0};
  std::uint32_t packet_width_{0};
  std::uint32_t packet_height_{0};

  std::unique_ptr<EventDecoderFactory> decoder_factory_;
  std::unique_ptr<EventTensorCudaBackend> cuda_backend_;
  std::vector<Metavision::EventCD> decoded_packet_events_;
  std::vector<CudaEvent> cuda_pending_events_;
  std::deque<Metavision::EventCD> window_events_;
  std::vector<float> tensor_buffer_;
  std::vector<std::vector<float>> staging_buffers_;
  std::vector<cudaEvent_t> staging_events_;
  std::vector<bool> staging_event_pending_;
  std::vector<bool> staging_buffer_pinned_;
  std::size_t next_staging_buffer_{0};
  Timestamp next_publish_us_{0};
  Timestamp publish_schedule_origin_us_{0};
  Timestamp next_publish_target_us_{0};
  Timestamp previous_window_end_us_{0};
  Timestamp last_event_us_{0};
  std::string frame_id_;
  std::chrono::steady_clock::time_point last_cuda_flush_time_;
  std::chrono::steady_clock::time_point last_event_arrival_time_;
  std::chrono::steady_clock::time_point last_event_sensor_update_time_;
  bool has_last_event_arrival_{false};
  bool has_last_event_sensor_update_{false};
  std::chrono::steady_clock::time_point in_flight_started_;
  std::int64_t in_flight_timestamp_ns_{0};
  std::int64_t last_snapshot_event_timestamp_us_{0};
  std::int64_t last_snapshot_header_timestamp_ns_{0};
  std::int64_t sensor_to_ros_offset_ns_{0};
  bool has_sensor_to_ros_offset_{false};
  bool cuda_ring_dirty_{false};
  bool inference_in_flight_{false};
  std::uint64_t cuda_events_since_snapshot_{0};

  nvidia::isaac_ros::common::CudaStreamPtr cuda_stream_;
  rclcpp::Subscription<EventPacket>::SharedPtr event_subscription_;
  rclcpp::Subscription<TensorList>::SharedPtr inference_feedback_subscription_;
  rclcpp::Publisher<TensorList>::SharedPtr tensor_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr cuda_timer_;

  std::atomic<std::uint64_t> received_packets_{0};
  std::atomic<std::uint64_t> decoded_events_{0};
  std::atomic<std::uint64_t> decode_calls_{0};
  std::atomic<std::uint64_t> decode_time_ns_{0};
  std::atomic<std::uint64_t> decode_time_max_ns_{0};
  std::atomic<std::uint64_t> packet_process_time_ns_{0};
  std::atomic<std::uint64_t> packet_process_time_max_ns_{0};
  std::atomic<std::uint64_t> decode_errors_{0};
  std::atomic<std::uint64_t> publish_errors_{0};
  std::atomic<std::uint64_t> inference_feedback_count_{0};
  std::atomic<std::uint64_t> stale_feedback_count_{0};
  std::atomic<std::uint64_t> watchdog_timeout_count_{0};
  std::atomic<std::uint64_t> fixed_rate_skipped_windows_{0};
  std::atomic<std::uint64_t> cuda_flush_count_{0};
  std::atomic<std::uint64_t> cuda_flush_events_{0};
  std::atomic<std::uint64_t> cuda_flush_time_ns_{0};
  std::atomic<std::uint64_t> cuda_flush_time_max_ns_{0};
  std::atomic<std::uint64_t> events_per_snapshot_gauge_{0};
  std::atomic<std::uint64_t> events_per_snapshot_max_{0};
  std::atomic<std::uint64_t> inference_round_trip_time_ns_{0};
  std::atomic<std::uint64_t> inference_round_trip_time_max_ns_{0};
  std::atomic<std::uint64_t> output_sensor_age_ns_{0};
  std::atomic<std::uint64_t> output_sensor_age_max_ns_{0};
  std::atomic<std::uint64_t> output_sensor_age_samples_{0};
  std::atomic<std::uint64_t> out_of_bounds_events_{0};
  std::atomic<std::uint64_t> out_of_order_events_{0};
  std::atomic<std::uint64_t> dropped_out_of_order_events_{0};
  std::atomic<std::uint64_t> timestamp_reset_count_{0};
  std::atomic<std::uint64_t> backward_jump_max_us_{0};
  std::atomic<std::uint64_t> published_tensors_{0};
  std::atomic<std::uint64_t> empty_windows_{0};
  std::atomic<std::uint64_t> full_windows_{0};
  std::atomic<std::uint64_t> incremental_windows_{0};
  std::atomic<std::uint64_t> represented_windows_{0};
  std::atomic<std::uint64_t> representation_time_ns_{0};
  std::atomic<std::uint64_t> representation_time_max_ns_{0};
  std::atomic<std::uint64_t> transfer_time_ns_{0};
  std::atomic<std::uint64_t> transfer_time_max_ns_{0};
  std::atomic<std::uint64_t> queued_events_gauge_{0};
  std::chrono::steady_clock::time_point last_statistics_time_;
};

}  // namespace openeb_ros2

#endif  // OPENEB_ROS2__EVENT_TENSOR_ENCODER_NODE_HPP_
