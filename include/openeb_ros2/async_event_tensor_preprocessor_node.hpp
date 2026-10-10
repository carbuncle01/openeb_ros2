#include "openeb_ros2/tensor_timing.hpp"
#ifndef OPENEB_ROS2__ASYNC_EVENT_TENSOR_PREPROCESSOR_NODE_HPP_
#define OPENEB_ROS2__ASYNC_EVENT_TENSOR_PREPROCESSOR_NODE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include "cuda_runtime_api.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "event_camera_codecs/decoder_factory.h"
#include "event_camera_codecs/event_processor.h"
#include "event_camera_msgs/msg/event_packet.hpp"
#include "isaac_ros_common/cuda_stream.hpp"
#include "openeb_ros2/tensor_output.hpp"
#include "isaac_ros_tensor_msgs/msg/tensor_list.hpp"
#include "openeb_ros2/event_tensor_cuda_backend.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openeb_ros2
{

// A single ROS node with isolated receive, decode, and GPU/snapshot stages.
// Only the raw packet callback runs on the ROS executor. Heavy work is kept on
// private workers so an event burst cannot monopolize the component container.
class AsyncEventTensorPreprocessorNode
  : public rclcpp::Node,
    public event_camera_codecs::EventProcessor
{
public:
  explicit AsyncEventTensorPreprocessorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AsyncEventTensorPreprocessorNode() override;

  void eventCD(
    std::uint64_t sensor_time, std::uint16_t x, std::uint16_t y,
    std::uint8_t polarity) override;
  bool eventExtTrigger(
    std::uint64_t sensor_time, std::uint8_t edge, std::uint8_t id) override;
  void finished() override;
  void rawData(const char * data, std::size_t size) override;

  // Direct input transfers ownership; caller must not retain the SDK buffer.
  void ingest(event_camera_msgs::msg::EventPacket::UniquePtr packet) { on_packet(std::move(packet)); }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr configuration_guard_;
  using EventPacket = event_camera_msgs::msg::EventPacket;
  using TensorList = isaac_ros_tensor_msgs::msg::TensorList;
  using DecoderFactory =
    event_camera_codecs::DecoderFactory<EventPacket, AsyncEventTensorPreprocessorNode>;
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct PacketWork
  {
    EventPacket::UniquePtr packet;
    SteadyTime enqueued_at;
  };

  struct DecodedWork
  {
    std::unique_ptr<std::vector<CudaEvent>> events;
    std::size_t offset{0U};
    std::string frame_id;
    std::int64_t sensor_to_ros_offset_ns{0};
    SteadyTime packet_enqueued_at;
    SteadyTime decoded_at;
    bool reset_before{false};
  };

  void on_packet(EventPacket::UniquePtr packet);
  void decode_loop();
  void gpu_loop();
  void decode_packet(PacketWork work);
  std::unique_ptr<std::vector<CudaEvent>> acquire_decode_buffer();
  void release_decode_buffer(std::unique_ptr<std::vector<CudaEvent>> buffer);
  void enqueue_decoded(DecodedWork work);
  bool pop_decoded(DecodedWork & work);
  void apply_chunk(DecodedWork & work);
  void publish_snapshot(std::int64_t window_end_us, SteadyTime scheduled_at);
  void publish_diagnostics();
  void request_decode_reset();
  void rebuild_coordinate_luts(std::uint32_t packet_width, std::uint32_t packet_height);
  std::int64_t ros_timestamp_ns(std::int64_t sensor_timestamp_us) const;
  std::int64_t aligned_window_end(std::int64_t target_us) const;

  std::int64_t bins_{10};
  std::int64_t width_{212};
  std::int64_t height_{120};
  std::int64_t window_us_{40000};
  std::int64_t stride_us_{4000};
  std::int64_t output_period_us_{4000};
  std::int64_t timestamp_backward_tolerance_us_{4000};
  std::size_t channels_{20U};
  std::size_t tensor_bytes_{0U};
  std::size_t packet_queue_capacity_{64U};
  std::size_t decoded_queue_capacity_{64U};
  std::size_t gpu_chunk_events_{8192U};
  std::size_t decode_buffer_initial_events_{16384U};
  std::size_t decode_buffer_pool_capacity_{4U};
  std::size_t decode_buffer_pool_max_events_{524288U};
  double max_queue_age_ms_{20.0};
  std::string polarity_mode_{"separate"};
  std::string polarity_layout_{"polarity_major"};
  std::string tensor_name_{"input_tensor"};
  std::vector<double> channel_mean_;
  std::vector<double> channel_stddev_;
  double deadline_ms_{4.0};
  double statistics_interval_s_{1.0};
  bool publish_empty_{true};
  bool debug_{false};

  std::unique_ptr<DecoderFactory> decoder_factory_;
  std::unique_ptr<EventTensorCudaBackend> cuda_backend_;
  nvidia::isaac_ros::common::CudaStreamPtr cuda_stream_;

  rclcpp::Subscription<EventPacket>::SharedPtr event_subscription_;
  std::unique_ptr<TensorTiming> tensor_timing_;
  rclcpp::Publisher<TensorList>::SharedPtr tensor_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::atomic<bool> stopping_{false};
  std::thread decode_thread_;
  std::thread gpu_thread_;
  std::mutex packet_mutex_;
  std::condition_variable packet_cv_;
  std::deque<PacketWork> packet_queue_;
  std::mutex decoded_mutex_;
  std::condition_variable decoded_cv_;
  std::deque<DecodedWork> decoded_queue_;
  std::mutex decode_buffer_pool_mutex_;
  std::vector<std::unique_ptr<std::vector<CudaEvent>>> decode_buffer_pool_;

  // Decode-thread-owned scratch and timestamp state used by EventProcessor callbacks.
  std::unique_ptr<std::vector<CudaEvent>> decode_scratch_;
  std::vector<std::uint16_t> output_x_lut_;
  std::vector<std::uint16_t> output_y_lut_;
  std::uint32_t packet_width_{0U};
  std::uint32_t packet_height_{0U};
  std::int64_t decode_last_event_us_{0};
  bool decode_reset_pending_{false};

  // GPU-thread-owned state.
  std::string frame_id_;
  std::int64_t sensor_to_ros_offset_ns_{0};
  std::int64_t publish_schedule_origin_us_{0};
  std::int64_t next_window_target_us_{0};
  std::int64_t next_window_end_us_{0};
  std::int64_t last_header_timestamp_ns_{0};
  SteadyTime next_snapshot_at_{};
  SteadyTime last_publish_at_{};
  std::uint64_t applied_event_version_{0U};
  std::uint64_t snapshot_event_version_{0U};
  std::uint64_t events_since_snapshot_{0U};

  std::chrono::steady_clock::time_point last_statistics_time_;
  std::atomic<std::int64_t> last_packet_arrival_steady_ns_{0};
  std::atomic<std::int64_t> last_applied_event_ros_ns_{0};

  std::atomic<std::uint64_t> received_packets_{0};
  std::atomic<std::uint64_t> received_packets_total_{0};
  std::atomic<bool> received_sequence_initialized_{false};
  std::atomic<std::uint64_t> last_received_sequence_{0};
  std::atomic<std::uint64_t> input_sequence_gap_occurrences_{0};
  std::atomic<std::uint64_t> input_sequence_missing_packets_{0};
  std::atomic<std::uint64_t> input_sequence_gap_max_{0};
  std::atomic<std::uint64_t> input_sequence_reorders_{0};
  std::atomic<std::uint64_t> packet_interarrival_samples_{0};
  std::atomic<std::uint64_t> packet_interarrival_ns_{0};
  std::atomic<std::uint64_t> packet_interarrival_max_ns_{0};
  std::atomic<std::uint64_t> packet_interarrival_over_output_period_{0};
  std::atomic<std::uint64_t> enqueued_packets_{0};
  std::atomic<std::uint64_t> dropped_packet_queue_packets_{0};
  std::atomic<std::uint64_t> dropped_packet_queue_bytes_{0};
  std::atomic<std::uint64_t> stale_packet_queue_packets_{0};
  std::atomic<std::uint64_t> packet_callback_time_ns_{0};
  std::atomic<std::uint64_t> packet_callback_time_max_ns_{0};
  std::atomic<std::uint64_t> decoded_packets_{0};
  std::atomic<std::uint64_t> decoded_events_{0};
  std::atomic<std::uint64_t> decode_errors_{0};
  std::atomic<std::uint64_t> decode_time_ns_{0};
  std::atomic<std::uint64_t> decode_time_max_ns_{0};
  std::atomic<std::uint64_t> decode_events_at_time_max_{0};
  std::atomic<std::uint64_t> decode_packet_events_max_{0};
  std::atomic<std::uint64_t> decode_time_at_events_max_ns_{0};
  std::atomic<std::uint64_t> decode_correlation_samples_{0};
  std::atomic<std::uint64_t> decode_correlation_events_sum_{0};
  std::atomic<std::uint64_t> decode_correlation_time_ns_sum_{0};
  std::atomic<std::uint64_t> decode_correlation_events_squared_sum_{0};
  std::atomic<std::uint64_t> decode_correlation_time_ns_squared_sum_{0};
  std::atomic<std::uint64_t> decode_correlation_cross_sum_{0};
  std::atomic<std::uint64_t> decode_buffer_growth_packets_{0};
  std::atomic<std::uint64_t> decode_buffer_growth_events_{0};
  std::atomic<std::uint64_t> decode_buffer_growth_events_max_{0};
  std::atomic<std::uint64_t> decode_buffer_capacity_before_events_{0};
  std::atomic<std::uint64_t> decode_buffer_capacity_after_events_max_{0};
  std::atomic<std::uint64_t> decode_buffer_pool_acquires_{0};
  std::atomic<std::uint64_t> decode_buffer_pool_hits_{0};
  std::atomic<std::uint64_t> decode_buffer_pool_misses_{0};
  std::atomic<std::uint64_t> decode_buffer_pool_returns_{0};
  std::atomic<std::uint64_t> decode_buffer_pool_discarded_full_{0};
  std::atomic<std::uint64_t> decode_buffer_pool_discarded_oversize_{0};
  std::atomic<std::uint64_t> decode_buffer_pool_depth_{0};
  std::atomic<std::uint64_t> decode_buffer_pool_depth_max_{0};
  std::atomic<std::uint64_t> decode_buffer_pool_retained_events_{0};
  std::atomic<std::uint64_t> decode_service_calls_{0};
  std::atomic<std::uint64_t> decode_service_time_ns_{0};
  std::atomic<std::uint64_t> decode_service_time_max_ns_{0};
  std::atomic<std::uint64_t> decode_handoff_time_ns_{0};
  std::atomic<std::uint64_t> decode_handoff_time_max_ns_{0};
  std::atomic<std::uint64_t> decode_thread_cpu_time_ns_{0};
  std::atomic<std::uint64_t> decode_thread_cpu_time_max_ns_{0};
  std::atomic<std::uint64_t> decode_scheduling_delay_ns_{0};
  std::atomic<std::uint64_t> decode_scheduling_delay_max_ns_{0};
  std::atomic<std::uint64_t> decode_cpu_migrations_{0};
  std::atomic<int> decode_last_cpu_{-1};
  std::atomic<std::uint64_t> decode_queue_wait_ns_{0};
  std::atomic<std::uint64_t> decode_queue_wait_max_ns_{0};
  std::atomic<std::uint64_t> dropped_decoded_batches_{0};
  std::atomic<std::uint64_t> dropped_decoded_events_{0};
  std::atomic<std::uint64_t> stale_decoded_batches_{0};
  std::atomic<std::uint64_t> stale_decoded_events_{0};
  std::atomic<std::uint64_t> gpu_updates_{0};
  std::atomic<std::uint64_t> gpu_events_{0};
  std::atomic<std::uint64_t> gpu_update_time_ns_{0};
  std::atomic<std::uint64_t> gpu_update_time_max_ns_{0};
  std::atomic<std::uint64_t> gpu_queue_wait_ns_{0};
  std::atomic<std::uint64_t> gpu_queue_wait_max_ns_{0};
  std::atomic<std::uint64_t> published_tensors_{0};
  std::atomic<std::uint64_t> publish_errors_{0};
  std::atomic<std::uint64_t> tensor_allocation_errors_{0};
  std::atomic<std::uint64_t> snapshot_time_ns_{0};
  std::atomic<std::uint64_t> snapshot_time_max_ns_{0};
  std::atomic<std::uint64_t> snapshot_lateness_ns_{0};
  std::atomic<std::uint64_t> snapshot_lateness_max_ns_{0};
  std::atomic<std::uint64_t> publish_interval_ns_{0};
  std::atomic<std::uint64_t> publish_interval_max_ns_{0};
  std::atomic<std::uint64_t> publish_interval_samples_{0};
  std::atomic<std::uint64_t> deadline_misses_{0};
  std::atomic<std::uint64_t> skipped_windows_{0};
  std::atomic<std::uint64_t> reused_snapshots_{0};
  std::atomic<std::uint64_t> events_per_snapshot_{0};
  std::atomic<std::uint64_t> events_per_snapshot_max_{0};
  std::atomic<std::uint64_t> out_of_bounds_events_{0};
  std::atomic<std::uint64_t> out_of_order_events_{0};
  std::atomic<std::uint64_t> dropped_out_of_order_events_{0};
  std::atomic<std::uint64_t> timestamp_resets_{0};
  std::atomic<std::uint64_t> backward_jump_max_us_{0};
  std::atomic<std::uint64_t> packet_queue_depth_{0};
  std::atomic<std::uint64_t> packet_queue_depth_max_{0};
  std::atomic<std::uint64_t> decoded_queue_depth_{0};
  std::atomic<std::uint64_t> decoded_queue_depth_max_{0};
  std::atomic<std::uint64_t> coordinate_lut_rebuilds_{0};
  std::atomic<std::uint64_t> state_age_ns_{0};
  std::atomic<std::uint64_t> state_age_max_ns_{0};
  std::atomic<std::uint64_t> window_end_age_ns_{0};
  std::atomic<std::uint64_t> window_end_age_max_ns_{0};
  std::atomic<std::uint64_t> schedule_initial_lag_ns_{0};
};

}  // namespace openeb_ros2

#endif  // OPENEB_ROS2__ASYNC_EVENT_TENSOR_PREPROCESSOR_NODE_HPP_
