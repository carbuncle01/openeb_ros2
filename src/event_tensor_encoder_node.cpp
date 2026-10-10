#include "openeb_ros2/event_tensor_encoder_node.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "openeb_ros2/tensor_output.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"

namespace openeb_ros2
{
namespace
{

void update_max(std::atomic<std::uint64_t> & maximum, const std::uint64_t value)
{
  auto current = maximum.load(std::memory_order_relaxed);
  while (
    current < value &&
    !maximum.compare_exchange_weak(current, value, std::memory_order_relaxed))
  {
  }
}

diagnostic_msgs::msg::KeyValue diagnostic_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

template<typename T>
diagnostic_msgs::msg::KeyValue diagnostic_number(std::string key, const T value)
{
  return diagnostic_value(std::move(key), std::to_string(value));
}

double milliseconds(const std::uint64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1.0e6;
}

}  // namespace

EventTensorEncoderNode::EventTensorEncoderNode(const rclcpp::NodeOptions & options)
: Node("event_tensor_encoder", options)
{
  bins_ = declare_parameter<std::int64_t>("bins", 10);
  width_ = declare_parameter<std::int64_t>("width", 212);
  height_ = declare_parameter<std::int64_t>("height", 120);
  const auto window_ms = declare_parameter<double>("window_ms", 40.0);
  const auto stride_ms = declare_parameter<double>("stride_ms", 4.0);
  output_rate_hz_ = declare_parameter<double>("output_rate_hz", 0.0);
  polarity_mode_ = declare_parameter<std::string>("polarity_mode", "separate");
  polarity_layout_ = declare_parameter<std::string>("polarity_layout", "polarity_major");
  temporal_interpolation_ =
    declare_parameter<std::string>("temporal_interpolation", "none");
  incremental_mode_ = declare_parameter<std::string>("incremental_mode", "auto");
  representation_backend_ = declare_parameter<std::string>("representation_backend", "cpu");
  inference_policy_ = declare_parameter<std::string>("inference_policy", "periodic");
  cuda_update_us_ = declare_parameter<std::int64_t>("cuda_update_us", 1000);
  timestamp_backward_tolerance_us_ = declare_parameter<std::int64_t>(
    "timestamp_backward_tolerance_us", 4000);
  const auto cuda_events_per_transfer =
    declare_parameter<std::int64_t>("cuda_events_per_transfer", 8192);
  inference_watchdog_ms_ = declare_parameter<double>("inference_watchdog_ms", 100.0);
  tensor_name_ = declare_parameter<std::string>("tensor_name", "input_tensor");
  channel_mean_ = declare_parameter<std::vector<double>>(
    "channel_mean", std::vector<double>{0.0});
  channel_stddev_ = declare_parameter<std::vector<double>>(
    "channel_stddev", std::vector<double>{1.0});
  publish_empty_ = declare_parameter<bool>("publish_empty", true);
  use_pinned_host_memory_ = declare_parameter<bool>("use_pinned_host_memory", true);
  debug_ = declare_parameter<bool>("debug", false);
  statistics_interval_s_ = declare_parameter<double>("statistics_interval_s", 1.0);
  const auto subscription_depth = declare_parameter<std::int64_t>("subscription_depth", 8);
  const auto publisher_depth = declare_parameter<std::int64_t>("publisher_depth", 4);
  const auto staging_buffer_count_parameter =
    declare_parameter<std::int64_t>("staging_buffer_count", 8);
  const auto diagnostics_topic = declare_parameter<std::string>(
    "diagnostics_topic", "tensor_diagnostics");

  if (bins_ <= 0 || width_ <= 0 || height_ <= 0) {
    throw std::invalid_argument("bins, width, and height must be positive");
  }
  if (
    width_ > std::numeric_limits<unsigned short>::max() ||
    height_ > std::numeric_limits<unsigned short>::max())
  {
    throw std::invalid_argument("width and height exceed the event coordinate range");
  }
  if (
    !std::isfinite(window_ms) || !std::isfinite(stride_ms) ||
    !std::isfinite(output_rate_hz_) || output_rate_hz_ < 0.0 || output_rate_hz_ > 1000.0 ||
    window_ms <= 0.0 || stride_ms <= 0.0 ||
    window_ms >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1000.0 ||
    stride_ms >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1000.0)
  {
    throw std::invalid_argument("window_ms and stride_ms must be finite and positive");
  }
  if (polarity_mode_ != "signed" && polarity_mode_ != "separate") {
    throw std::invalid_argument("polarity_mode must be 'signed' or 'separate'");
  }
  if (polarity_layout_ != "polarity_major" && polarity_layout_ != "time_major") {
    throw std::invalid_argument(
            "polarity_layout must be 'polarity_major' or 'time_major'");
  }
  if (temporal_interpolation_ != "none" && temporal_interpolation_ != "linear") {
    throw std::invalid_argument("temporal_interpolation must be 'none' or 'linear'");
  }
  if (
    incremental_mode_ != "auto" && incremental_mode_ != "off" &&
    incremental_mode_ != "require")
  {
    throw std::invalid_argument("incremental_mode must be 'auto', 'off', or 'require'");
  }
  if (representation_backend_ != "cpu" && representation_backend_ != "cuda") {
    throw std::invalid_argument("representation_backend must be 'cpu' or 'cuda'");
  }
  if (inference_policy_ != "periodic" && inference_policy_ != "consumer_driven") {
    throw std::invalid_argument("inference_policy must be 'periodic' or 'consumer_driven'");
  }
  if (representation_backend_ == "cuda" && temporal_interpolation_ != "none") {
    throw std::invalid_argument(
            "CUDA rolling representation currently requires temporal_interpolation=none; "
            "use representation_backend=cpu for linear interpolation");
  }
  if (
    subscription_depth <= 0 || publisher_depth <= 0 ||
    staging_buffer_count_parameter <= 0 || !std::isfinite(statistics_interval_s_) ||
    statistics_interval_s_ < 0.0 ||
    statistics_interval_s_ >=
    static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1000.0)
  {
    throw std::invalid_argument("queue depths/pool blocks must be positive");
  }
  if (
    cuda_update_us_ <= 0 || cuda_events_per_transfer <= 0 ||
    timestamp_backward_tolerance_us_ < 0 ||
    !std::isfinite(inference_watchdog_ms_) || inference_watchdog_ms_ <= 0.0)
  {
    throw std::invalid_argument("CUDA update, transfer size, and watchdog must be positive");
  }
  cuda_events_per_transfer_ = static_cast<std::size_t>(cuda_events_per_transfer);

  window_us_ = static_cast<std::int64_t>(std::llround(window_ms * 1000.0));
  stride_us_ = static_cast<std::int64_t>(std::llround(stride_ms * 1000.0));
  publish_period_us_ = output_rate_hz_ > 0.0 ?
    static_cast<std::int64_t>(std::llround(1.0e6 / output_rate_hz_)) : stride_us_;
  if (output_rate_hz_ > 0.0 && output_rate_hz_ > 1.0e6 / stride_us_ + 1.0e-9) {
    throw std::invalid_argument(
            "output_rate_hz cannot exceed the representation boundary rate 1e6/stride_us");
  }
  const auto polarities = polarity_mode_ == "separate" ? 2U : 1U;
  if (
    window_us_ <= 0 || stride_us_ <= 0 || publish_period_us_ <= 0 ||
    static_cast<std::uint64_t>(bins_) >
    static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) / polarities)
  {
    throw std::invalid_argument("rounded window/stride and tensor channels must be positive");
  }
  channels_ = static_cast<std::size_t>(bins_) * polarities;
  pixels_ = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  if (pixels_ > std::numeric_limits<std::size_t>::max() / channels_) {
    throw std::overflow_error("event tensor dimensions overflow size_t");
  }
  tensor_elements_ = channels_ * pixels_;
  if (tensor_elements_ > std::numeric_limits<std::size_t>::max() / sizeof(float)) {
    throw std::overflow_error("event tensor byte size overflows size_t");
  }
  tensor_bytes_ = tensor_elements_ * sizeof(float);
  tensor_buffer_.assign(tensor_elements_, 0.0F);

  if (representation_backend_ == "cuda") {
    if (stride_us_ > window_us_) {
      throw std::invalid_argument("CUDA fixed-rate representation requires stride <= window");
    }
    if (window_us_ % bins_ != 0) {
      throw std::invalid_argument("CUDA representation requires window_us divisible by bins");
    }
    const auto bin_width_us = window_us_ / bins_;
    if (stride_us_ % bin_width_us != 0) {
      throw std::invalid_argument(
              "CUDA fixed-rate representation requires stride_us to be an integer multiple "
              "of window_us/bins");
    }
  }

  const auto validate_channel_parameter = [this](
      const std::vector<double> & values, const char * name, const bool positive) {
      if (values.size() != 1U && values.size() != channels_) {
        throw std::invalid_argument(
                std::string(name) + " must contain one value or one value per channel");
      }
      for (const auto value : values) {
        if (!std::isfinite(value) || (positive && value <= 0.0)) {
          throw std::invalid_argument(std::string(name) + " contains an invalid value");
        }
      }
    };
  validate_channel_parameter(channel_mean_, "channel_mean", false);
  validate_channel_parameter(channel_stddev_, "channel_stddev", true);
  normalization_identity_ = channel_mean_.size() == 1U && channel_stddev_.size() == 1U &&
    channel_mean_.front() == 0.0 && channel_stddev_.front() == 1.0;

  use_incremental_ = representation_backend_ == "cpu" &&
    incremental_mode_ != "off" && incremental_eligible();
  shift_bins_ = use_incremental_ ? incremental_shift_bins() : 0U;
  if (
    representation_backend_ == "cpu" && incremental_mode_ == "require" &&
    !use_incremental_)
  {
    throw std::invalid_argument(
            "incremental mode requires interpolation=none, overlapping windows, "
            "and stride_us to be an integer multiple of window_us/bins");
  }
  if (
    representation_backend_ == "cpu" && incremental_mode_ == "auto" &&
    !use_incremental_)
  {
    RCLCPP_INFO(
      get_logger(),
      "Incremental event-bin reuse is not exact for this configuration; using full rebuilds");
  }

  cuda_stream_ = nvidia::isaac_ros::common::createCudaStream("EventTensorEncoderNode");

  if (representation_backend_ == "cuda") {
    cuda_backend_ = std::make_unique<EventTensorCudaBackend>(
      static_cast<std::size_t>(width_), static_cast<std::size_t>(height_),
      static_cast<std::size_t>(bins_), polarity_mode_ == "separate",
      polarity_layout_ == "polarity_major", window_us_, cuda_events_per_transfer_,
      channel_mean_, channel_stddev_);
    cuda_pending_events_.reserve(cuda_events_per_transfer_ * 2U);
  } else {
    const auto staging_buffer_count = static_cast<std::size_t>(staging_buffer_count_parameter);
    staging_buffers_.resize(staging_buffer_count);
    staging_events_.resize(staging_buffer_count, nullptr);
    staging_event_pending_.resize(staging_buffer_count, false);
    staging_buffer_pinned_.resize(staging_buffer_count, false);
    for (std::size_t index = 0; index < staging_buffer_count; ++index) {
      staging_buffers_[index].assign(tensor_elements_, 0.0F);
      CHECK_CUDA_ERROR(
        cudaEventCreateWithFlags(&staging_events_[index], cudaEventDisableTiming),
        "Failed to create event tensor staging event");
      if (use_pinned_host_memory_) {
        const auto result = cudaHostRegister(
          staging_buffers_[index].data(), tensor_bytes_, cudaHostRegisterPortable);
        if (result == cudaSuccess) {
          staging_buffer_pinned_[index] = true;
        } else {
          RCLCPP_WARN(
            get_logger(), "Could not pin host staging buffer %zu: %s; using pageable memory",
            index, cudaGetErrorString(result));
          (void)cudaGetLastError();
        }
      }
    }
  }

  decoder_factory_ = std::make_unique<EventDecoderFactory>();
  const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(
      static_cast<std::size_t>(subscription_depth))).best_effort().durability_volatile();
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(
      static_cast<std::size_t>(publisher_depth))).reliable().durability_volatile();
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  rclcpp::PublisherOptions publisher_options;
  publisher_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  if (declare_parameter<bool>("subscribe_packets", true)) {
    event_subscription_ = create_subscription<EventPacket>(
      "events", input_qos,
      std::bind(&EventTensorEncoderNode::on_packet, this, std::placeholders::_1),
      subscription_options);
  }
  tensor_timing_ = std::make_unique<TensorTiming>(*this);
  tensor_publisher_ = create_publisher<TensorList>("tensor", output_qos, publisher_options);
  if (representation_backend_ == "cuda" && inference_policy_ == "consumer_driven") {
    inference_feedback_subscription_ = create_subscription<TensorList>(
      "tensor_feedback", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      std::bind(&EventTensorEncoderNode::on_inference_output, this, std::placeholders::_1),
      subscription_options);
  }
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);

  last_statistics_time_ = std::chrono::steady_clock::now();
  last_cuda_flush_time_ = last_statistics_time_;
  if (statistics_interval_s_ > 0.0) {
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max<std::int64_t>(
          1, static_cast<std::int64_t>(statistics_interval_s_ * 1000.0))),
      std::bind(&EventTensorEncoderNode::publish_diagnostics, this));
  }
  if (representation_backend_ == "cuda") {
    cuda_timer_ = create_wall_timer(
      std::chrono::microseconds(cuda_update_us_),
      std::bind(&EventTensorEncoderNode::on_cuda_timer, this));
  }

  const auto reported_shift_bins = representation_backend_ == "cuda" ?
    static_cast<std::size_t>(stride_us_ / (window_us_ / bins_)) : shift_bins_;
  const auto reported_incremental = use_incremental_ ||
    (representation_backend_ == "cuda" && stride_us_ < window_us_);
  RCLCPP_INFO(
    get_logger(),
    "Event tensor encoder: shape=[1,%zu,%ld,%ld], bins=%ld, window=%.3fms, "
    "stride=%.3fms, output_rate=%.3fHz, backend=%s, policy=%s, interpolation=%s, "
    "incremental=%s (shift=%zu), timestamp_backward_tolerance=%ldus",
    channels_, height_, width_, bins_, window_ms, stride_ms,
    output_rate_hz_ > 0.0 ? output_rate_hz_ : 1.0e6 / stride_us_,
    representation_backend_.c_str(), inference_policy_.c_str(), temporal_interpolation_.c_str(),
    reported_incremental ? "enabled" : "disabled", reported_shift_bins,
    timestamp_backward_tolerance_us_);
  configuration_guard_ = add_on_set_parameters_callback(
    [](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() != "use_sim_time") {
          result.successful = false;
          result.reason = "Tensor configuration is startup-only; restart the pipeline";
          break;
        }
      }
      return result;
    });
}

EventTensorEncoderNode::~EventTensorEncoderNode()
{
  if (cuda_stream_) {
    (void)cudaStreamSynchronize(*cuda_stream_);
  }
  cuda_backend_.reset();
  for (std::size_t index = 0; index < staging_buffers_.size(); ++index) {
    if (staging_buffer_pinned_[index]) {
      (void)cudaHostUnregister(staging_buffers_[index].data());
    }
    if (staging_events_[index] != nullptr) {
      (void)cudaEventDestroy(staging_events_[index]);
    }
  }
}

bool EventTensorEncoderNode::incremental_eligible() const
{
  if (temporal_interpolation_ != "none" || stride_us_ >= window_us_) {
    return false;
  }
  if (window_us_ % bins_ != 0) {
    return false;
  }
  const auto bin_width_us = window_us_ / bins_;
  return bin_width_us > 0 && stride_us_ % bin_width_us == 0 &&
         stride_us_ / bin_width_us < bins_;
}

std::size_t EventTensorEncoderNode::incremental_shift_bins() const
{
  return static_cast<std::size_t>(stride_us_ / (window_us_ / bins_));
}

std::int64_t EventTensorEncoderNode::ros_timestamp_ns(
  const Timestamp sensor_timestamp_us) const
{
  const auto sensor_ns = static_cast<std::int64_t>(sensor_timestamp_us) * 1000LL;
  return has_sensor_to_ros_offset_ ? sensor_ns + sensor_to_ros_offset_ns_ : sensor_ns;
}

void EventTensorEncoderNode::initialize_publish_schedule(const Timestamp first_event_us)
{
  publish_schedule_origin_us_ = first_event_us + window_us_;
  next_publish_target_us_ = publish_schedule_origin_us_;
  next_publish_us_ = publish_schedule_origin_us_;
}

void EventTensorEncoderNode::advance_publish_schedule()
{
  next_publish_target_us_ += publish_period_us_;
  const auto target_offset = next_publish_target_us_ - publish_schedule_origin_us_;
  const auto stride_steps = std::max<std::int64_t>(
    1, static_cast<std::int64_t>(std::llround(
      static_cast<double>(target_offset) / static_cast<double>(stride_us_))));
  auto candidate = publish_schedule_origin_us_ + stride_steps * stride_us_;
  if (candidate <= next_publish_us_) {
    candidate = next_publish_us_ + stride_us_;
  }
  next_publish_us_ = candidate;
}

void EventTensorEncoderNode::eventCD(
  const std::uint64_t sensor_time, const std::uint16_t x, const std::uint16_t y,
  const std::uint8_t polarity)
{
  if (packet_width_ == 0U || packet_height_ == 0U || x >= packet_width_ || y >= packet_height_) {
    out_of_bounds_events_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const auto output_x = std::min<std::uint32_t>(
    static_cast<std::uint32_t>(width_ - 1),
    static_cast<std::uint32_t>(static_cast<std::uint64_t>(x) * width_ / packet_width_));
  const auto output_y = std::min<std::uint32_t>(
    static_cast<std::uint32_t>(height_ - 1),
    static_cast<std::uint32_t>(static_cast<std::uint64_t>(y) * height_ / packet_height_));
  decoded_packet_events_.emplace_back(
    static_cast<unsigned short>(output_x), static_cast<unsigned short>(output_y),
    static_cast<short>(polarity), static_cast<Timestamp>(sensor_time / 1000ULL));
}

bool EventTensorEncoderNode::eventExtTrigger(
  const std::uint64_t, const std::uint8_t, const std::uint8_t)
{
  return true;
}

void EventTensorEncoderNode::finished()
{
}

void EventTensorEncoderNode::rawData(const char *, const std::size_t)
{
}

void EventTensorEncoderNode::on_packet(EventPacket::UniquePtr packet)
{
  std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
  if (!packet || packet->events.empty()) {
    return;
  }
  received_packets_.fetch_add(1, std::memory_order_relaxed);
  const auto decode_start = std::chrono::steady_clock::now();
  last_event_arrival_time_ = decode_start;
  has_last_event_arrival_ = true;
  decoded_packet_events_.clear();
  if (
    packet_width_ != 0U && packet_height_ != 0U &&
    (packet_width_ != packet->width || packet_height_ != packet->height))
  {
    reset_state("event packet geometry changed");
  }
  packet_width_ = packet->width;
  packet_height_ = packet->height;
  try {
    auto * decoder = decoder_factory_->getInstance(*packet);
    if (!decoder) {
      throw std::runtime_error("no decoder for event packet encoding '" + packet->encoding + "'");
    }
    while (decoder->decode(*packet, this)) {
    }
  } catch (const std::exception & error) {
    decode_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Event packet decode failed: %s", error.what());
    decoder_factory_ = std::make_unique<EventDecoderFactory>();
    reset_state("decoder error");
    return;
  }
  const auto decode_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - decode_start).count());
  decoded_events_.fetch_add(decoded_packet_events_.size(), std::memory_order_relaxed);
  decode_calls_.fetch_add(1, std::memory_order_relaxed);
  decode_time_ns_.fetch_add(decode_ns, std::memory_order_relaxed);
  update_max(decode_time_max_ns_, decode_ns);
  frame_id_ = packet->header.frame_id;
  if (!decoded_packet_events_.empty()) {
    const auto packet_stamp_ns =
      static_cast<std::int64_t>(packet->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(packet->header.stamp.nanosec);
    sensor_to_ros_offset_ns_ = packet_stamp_ns -
      static_cast<std::int64_t>(decoded_packet_events_.back().t) * 1000LL;
    has_sensor_to_ros_offset_ = true;
  }
  process_events(decoded_packet_events_);
  const auto packet_process_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - decode_start).count());
  packet_process_time_ns_.fetch_add(packet_process_ns, std::memory_order_relaxed);
  update_max(packet_process_time_max_ns_, packet_process_ns);
}

void EventTensorEncoderNode::process_events(
  const std::vector<Metavision::EventCD> & events)
{
  if (representation_backend_ == "cuda") {
    for (const auto & event : events) {
      if (!accept_event_timestamp(event.t)) {
        continue;
      }
      if (next_publish_us_ == 0) {
        initialize_publish_schedule(event.t);
      }
      cuda_pending_events_.push_back(CudaEvent{
        static_cast<std::int64_t>(event.t), event.x, event.y,
        static_cast<std::uint8_t>(event.p != 0), {0, 0, 0}});
    }
    if (!events.empty()) {
      last_event_sensor_update_time_ = std::chrono::steady_clock::now();
      has_last_event_sensor_update_ = true;
    }
    queued_events_gauge_.store(cuda_pending_events_.size(), std::memory_order_relaxed);
    if (cuda_pending_events_.size() >= cuda_events_per_transfer_) {
      flush_cuda_events();
    }
    return;
  }

  for (const auto & event : events) {
    if (!accept_event_timestamp(event.t)) {
      continue;
    }
    if (next_publish_us_ == 0) {
      initialize_publish_schedule(event.t);
    }

    // Half-open windows [start, end) make histogram bins exactly reusable.
    while (event.t >= next_publish_us_) {
      publish_window(next_publish_us_, frame_id_);
      advance_publish_schedule();
    }

    if (event.x >= width_ || event.y >= height_) {
      out_of_bounds_events_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    window_events_.push_back(event);
  }
  const auto oldest_needed = next_publish_us_ - window_us_;
  while (!window_events_.empty() && window_events_.front().t < oldest_needed) {
    window_events_.pop_front();
  }
  queued_events_gauge_.store(window_events_.size(), std::memory_order_relaxed);
}

void EventTensorEncoderNode::flush_cuda_events()
{
  if (!cuda_backend_ || cuda_pending_events_.empty()) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const auto count = cuda_pending_events_.size();
  try {
    cuda_backend_->update(cuda_pending_events_, *cuda_stream_);
    cuda_pending_events_.clear();
    cuda_pending_events_.reserve(cuda_events_per_transfer_ * 2U);
    queued_events_gauge_.store(0, std::memory_order_relaxed);
    last_cuda_flush_time_ = std::chrono::steady_clock::now();
    cuda_ring_dirty_ = true;
    cuda_events_since_snapshot_ += count;
    const auto elapsed_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        last_cuda_flush_time_ - started).count());
    cuda_flush_count_.fetch_add(1, std::memory_order_relaxed);
    cuda_flush_events_.fetch_add(count, std::memory_order_relaxed);
    cuda_flush_time_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed);
    update_max(cuda_flush_time_max_ns_, elapsed_ns);

    if (inference_policy_ == "consumer_driven") {
      maybe_publish_cuda_snapshot(
        static_cast<Timestamp>(cuda_backend_->latest_window_end_us()));
    }
  } catch (const std::exception & error) {
    publish_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "CUDA event update failed: %s", error.what());
  }
}

void EventTensorEncoderNode::maybe_publish_cuda_snapshot(const Timestamp window_end_us)
{
  if (
    !cuda_backend_ || window_end_us <= 0 || !cuda_backend_->ready() ||
    (inference_policy_ == "consumer_driven" && !cuda_ring_dirty_) ||
    (inference_policy_ == "consumer_driven" && inference_in_flight_))
  {
    return;
  }

  const auto representation_start = std::chrono::steady_clock::now();
  try {
    tensor_msgs::msg::ExperimentalTensor tensor;
    std::vector<std::int64_t> shape{
      1, static_cast<std::int32_t>(channels_), static_cast<std::int32_t>(height_),
      static_cast<std::int32_t>(width_)};
    {
      auto write_handle = allocate_tensor(tensor, shape, *cuda_stream_);
      cuda_backend_->snapshot(
        reinterpret_cast<float *>(write_handle.get_ptr()), window_end_us, *cuda_stream_);
    }

    const auto timestamp_us = window_end_us;
    auto timestamp_ns = ros_timestamp_ns(window_end_us);
    timestamp_ns = std::max(
      timestamp_ns, last_snapshot_header_timestamp_ns_ + std::int64_t{1});
    std_msgs::msg::Header header;
    header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000LL);
    header.stamp.nanosec =
      static_cast<std::uint32_t>(timestamp_ns % 1000000000LL);
    header.frame_id = frame_id_;
    auto tensor_list = make_tensor_list(header, tensor_name_, std::move(tensor));

    const auto now_steady = std::chrono::steady_clock::now();
    const auto representation_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        now_steady - representation_start).count());
    representation_time_ns_.fetch_add(representation_ns, std::memory_order_relaxed);
    represented_windows_.fetch_add(1, std::memory_order_relaxed);
    update_max(representation_time_max_ns_, representation_ns);
    if (inference_policy_ == "consumer_driven") {
      inference_in_flight_ = true;
      in_flight_timestamp_ns_ = timestamp_ns;
      in_flight_started_ = now_steady;
    }
    last_snapshot_event_timestamp_us_ = timestamp_us;
    last_snapshot_header_timestamp_ns_ = timestamp_ns;
    events_per_snapshot_gauge_.store(cuda_events_since_snapshot_, std::memory_order_relaxed);
    update_max(events_per_snapshot_max_, cuda_events_since_snapshot_);
    cuda_events_since_snapshot_ = 0;
    cuda_ring_dirty_ = false;
    const auto publish_start = std::chrono::steady_clock::now();
    tensor_publisher_->publish(tensor_list);
    tensor_timing_->publish(header, window_end_us-window_us_, window_end_us,
      sensor_to_ros_offset_ns_, last_event_us_, tensor_name_, channels_, height_, width_);
    const auto publish_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - publish_start).count());
    transfer_time_ns_.fetch_add(publish_ns, std::memory_order_relaxed);
    update_max(transfer_time_max_ns_, publish_ns);
    published_tensors_.fetch_add(1, std::memory_order_relaxed);
  } catch (const std::exception & error) {
    inference_in_flight_ = false;
    cuda_ring_dirty_ = true;
    publish_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "CUDA event snapshot publish failed: %s", error.what());
  }
}

void EventTensorEncoderNode::on_inference_output(TensorList::ConstSharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
  if (!message || !inference_in_flight_) {
    stale_feedback_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const auto timestamp_ns =
    static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(message->header.stamp.nanosec);
  if (timestamp_ns != in_flight_timestamp_ns_) {
    stale_feedback_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const auto elapsed_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - in_flight_started_).count());
  inference_round_trip_time_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed);
  update_max(inference_round_trip_time_max_ns_, elapsed_ns);
  const auto sensor_age_ns = now().nanoseconds() - timestamp_ns;
  if (sensor_age_ns >= 0) {
    output_sensor_age_ns_.fetch_add(
      static_cast<std::uint64_t>(sensor_age_ns), std::memory_order_relaxed);
    update_max(output_sensor_age_max_ns_, static_cast<std::uint64_t>(sensor_age_ns));
    output_sensor_age_samples_.fetch_add(1, std::memory_order_relaxed);
  }
  inference_feedback_count_.fetch_add(1, std::memory_order_relaxed);
  inference_in_flight_ = false;
  maybe_publish_cuda_snapshot(
    static_cast<Timestamp>(cuda_backend_->latest_window_end_us()));
}

void EventTensorEncoderNode::on_cuda_timer()
{
  std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
  const auto now_steady = std::chrono::steady_clock::now();
  if (
    !cuda_pending_events_.empty() &&
    std::chrono::duration_cast<std::chrono::microseconds>(
      now_steady - last_cuda_flush_time_).count() >= cuda_update_us_)
  {
    flush_cuda_events();
  }
  if (
    inference_policy_ == "periodic" && cuda_backend_ && cuda_backend_->ready() &&
    next_publish_us_ > 0 && has_last_event_sensor_update_)
  {
    // Advance the event-time clock even when no CD events arrive. Publishing
    // the scheduled half-open window (instead of replaying the last tensor)
    // lets old bins decay to zero while keeping a deterministic output rate.
    const auto estimated_sensor_us = last_event_us_ + static_cast<Timestamp>(
      std::chrono::duration_cast<std::chrono::microseconds>(
        now_steady - last_event_sensor_update_time_).count());
    if (estimated_sensor_us >= next_publish_us_) {
      std::uint64_t due_windows = 0U;
      Timestamp publish_end_us = next_publish_us_;
      while (estimated_sensor_us >= next_publish_us_) {
        publish_end_us = next_publish_us_;
        advance_publish_schedule();
        ++due_windows;
      }
      if (due_windows > 1U) {
        fixed_rate_skipped_windows_.fetch_add(due_windows - 1U, std::memory_order_relaxed);
      }
      maybe_publish_cuda_snapshot(publish_end_us);
    }
  }
  if (
    inference_policy_ == "consumer_driven" && inference_in_flight_ &&
    std::chrono::duration<double, std::milli>(now_steady - in_flight_started_).count() >=
    inference_watchdog_ms_)
  {
    inference_in_flight_ = false;
    watchdog_timeout_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TensorRT feedback watchdog expired; waiting for the next updated event snapshot");
    maybe_publish_cuda_snapshot(
      static_cast<Timestamp>(cuda_backend_->latest_window_end_us()));
  }
}

void EventTensorEncoderNode::publish_window(
  const Timestamp window_end_us, const std::string & frame_id)
{
  const auto representation_start = std::chrono::steady_clock::now();
  const auto window_start_us = window_end_us - window_us_;
  while (!window_events_.empty() && window_events_.front().t < window_start_us) {
    window_events_.pop_front();
  }

  const bool incremental = use_incremental_ && previous_window_end_us_ != 0 &&
    window_end_us - previous_window_end_us_ == stride_us_;
  if (incremental) {
    build_incremental_histogram(window_start_us, window_end_us);
    incremental_windows_.fetch_add(1, std::memory_order_relaxed);
  } else {
    build_full_histogram(window_start_us, window_end_us);
    full_windows_.fetch_add(1, std::memory_order_relaxed);
  }
  previous_window_end_us_ = window_end_us;

  const bool empty = window_events_.empty();
  if (empty) {
    empty_windows_.fetch_add(1, std::memory_order_relaxed);
    if (!publish_empty_) {
      return;
    }
  }
  try {
    const auto staging_index = next_staging_buffer_;
    next_staging_buffer_ = (next_staging_buffer_ + 1U) % staging_buffers_.size();
    if (staging_event_pending_[staging_index]) {
      CHECK_CUDA_ERROR(
        cudaEventSynchronize(staging_events_[staging_index]),
        "Failed while waiting to reuse an event tensor staging buffer");
    }
    auto & staging_buffer = staging_buffers_[staging_index];
    prepare_staging_buffer(staging_buffer);
    const auto representation_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - representation_start).count());
    representation_time_ns_.fetch_add(representation_ns, std::memory_order_relaxed);
    represented_windows_.fetch_add(1, std::memory_order_relaxed);
    update_max(representation_time_max_ns_, representation_ns);

    const auto transfer_start = std::chrono::steady_clock::now();
    tensor_msgs::msg::ExperimentalTensor tensor;
    std::vector<std::int64_t> shape{
      1, static_cast<std::int32_t>(channels_), static_cast<std::int32_t>(height_),
      static_cast<std::int32_t>(width_)};
    {
      auto write_handle = allocate_tensor(tensor, shape, *cuda_stream_);
      CHECK_CUDA_ERROR(
        cudaMemcpyAsync(
          write_handle.get_ptr(), staging_buffer.data(), tensor_bytes_,
          cudaMemcpyHostToDevice, *cuda_stream_),
        "Failed to copy event tensor to CUDA memory");
      CHECK_CUDA_ERROR(
        cudaEventRecord(staging_events_[staging_index], *cuda_stream_),
        "Failed to record event tensor staging completion");
      staging_event_pending_[staging_index] = true;
      // Destroying the handle records the completion event consumed downstream.
    }

    const auto timestamp_ns = ros_timestamp_ns(window_end_us);
    std_msgs::msg::Header header;
    header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000LL);
    header.stamp.nanosec =
      static_cast<std::uint32_t>(timestamp_ns % 1000000000LL);
    header.frame_id = frame_id;
    auto tensor_list = make_tensor_list(header, tensor_name_, std::move(tensor));
    tensor_publisher_->publish(tensor_list);
    tensor_timing_->publish(header, window_end_us-window_us_, window_end_us,
      sensor_to_ros_offset_ns_, last_event_us_, tensor_name_, channels_, height_, width_);
    const auto transfer_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - transfer_start).count());
    transfer_time_ns_.fetch_add(transfer_ns, std::memory_order_relaxed);
    update_max(transfer_time_max_ns_, transfer_ns);
    published_tensors_.fetch_add(1, std::memory_order_relaxed);
  } catch (const std::exception & error) {
    (void)cudaStreamSynchronize(*cuda_stream_);
    publish_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Event tensor publish failed: %s", error.what());
  }
}

void EventTensorEncoderNode::build_full_histogram(
  const Timestamp window_start_us, const Timestamp window_end_us)
{
  std::fill(tensor_buffer_.begin(), tensor_buffer_.end(), 0.0F);
  for (const auto & event : window_events_) {
    if (event.t < window_start_us) {
      continue;
    }
    if (event.t >= window_end_us) {
      break;
    }
    if (temporal_interpolation_ == "linear") {
      accumulate_linear_event(event, window_start_us, window_end_us, tensor_buffer_);
    } else {
      accumulate_histogram_event(event, window_start_us, window_end_us, tensor_buffer_);
    }
  }
}

void EventTensorEncoderNode::build_incremental_histogram(
  const Timestamp window_start_us, const Timestamp window_end_us)
{
  const auto shift = shift_bins_;
  const auto bins = static_cast<std::size_t>(bins_);
  const auto shift_block = [this, shift, bins](float * block, const std::size_t planes_per_bin) {
    const auto removed_planes = shift * planes_per_bin;
    const auto total_planes = bins * planes_per_bin;
    const auto retained_values = (total_planes - removed_planes) * pixels_;
    std::memmove(
      block, block + removed_planes * pixels_, retained_values * sizeof(float));
    std::fill(block + retained_values, block + total_planes * pixels_, 0.0F);
  };
  if (polarity_mode_ == "separate" && polarity_layout_ == "polarity_major") {
    shift_block(tensor_buffer_.data(), 1U);
    shift_block(tensor_buffer_.data() + bins * pixels_, 1U);
  } else {
    shift_block(
      tensor_buffer_.data(),
      polarity_mode_ == "separate" ? 2U : 1U);
  }

  const auto new_data_start_us = window_end_us - stride_us_;
  const auto first_new_event = std::lower_bound(
    window_events_.cbegin(), window_events_.cend(), new_data_start_us,
    [](const Metavision::EventCD & event, const Timestamp timestamp) {
      return event.t < timestamp;
    });
  for (auto event_it = first_new_event; event_it != window_events_.cend(); ++event_it) {
    const auto & event = *event_it;
    if (event.t >= window_end_us) {
      break;
    }
    accumulate_histogram_event(event, window_start_us, window_end_us, tensor_buffer_);
  }
}

std::size_t EventTensorEncoderNode::channel_index(
  const std::size_t bin, const bool positive) const
{
  if (polarity_mode_ == "signed") {
    return bin;
  }
  const auto polarity = positive ? 0U : 1U;
  return polarity_layout_ == "polarity_major" ?
    polarity * static_cast<std::size_t>(bins_) + bin : bin * 2U + polarity;
}

void EventTensorEncoderNode::accumulate_histogram_event(
  const Metavision::EventCD & event, const Timestamp window_start_us,
  const Timestamp window_end_us, std::vector<float> & tensor) const
{
  if (event.t < window_start_us || event.t >= window_end_us) {
    return;
  }
  const auto elapsed = static_cast<std::uint64_t>(event.t - window_start_us);
  const auto bin = std::min<std::size_t>(
    static_cast<std::size_t>(bins_ - 1),
    static_cast<std::size_t>(elapsed * bins_ / window_us_));
  const auto pixel = static_cast<std::size_t>(event.y) * width_ + event.x;
  const auto channel = channel_index(bin, event.p != 0);
  const float value = polarity_mode_ == "signed" && event.p == 0 ? -1.0F : 1.0F;
  tensor[channel * pixels_ + pixel] += value;
}

void EventTensorEncoderNode::accumulate_linear_event(
  const Metavision::EventCD & event, const Timestamp window_start_us,
  const Timestamp window_end_us, std::vector<float> & tensor) const
{
  if (event.t < window_start_us || event.t >= window_end_us) {
    return;
  }
  const auto elapsed = static_cast<double>(event.t - window_start_us);
  const auto position = bins_ == 1 ? 0.0 :
    elapsed * static_cast<double>(bins_ - 1) / static_cast<double>(window_us_);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = std::min<std::size_t>(lower + 1U, bins_ - 1U);
  const float upper_weight = static_cast<float>(position - std::floor(position));
  const float lower_weight = 1.0F - upper_weight;
  const auto pixel = static_cast<std::size_t>(event.y) * width_ + event.x;
  const float polarity = polarity_mode_ == "signed" && event.p == 0 ? -1.0F : 1.0F;
  tensor[channel_index(lower, event.p != 0) * pixels_ + pixel] += polarity * lower_weight;
  if (upper != lower) {
    tensor[channel_index(upper, event.p != 0) * pixels_ + pixel] += polarity * upper_weight;
  }
}

void EventTensorEncoderNode::prepare_staging_buffer(std::vector<float> & output) const
{
  if (normalization_identity_) {
    std::copy(tensor_buffer_.cbegin(), tensor_buffer_.cend(), output.begin());
    return;
  }
  std::copy(tensor_buffer_.cbegin(), tensor_buffer_.cend(), output.begin());
  for (std::size_t channel = 0; channel < channels_; ++channel) {
    const auto mean = static_cast<float>(
      channel_mean_[channel_mean_.size() == 1U ? 0U : channel]);
    const auto stddev = static_cast<float>(
      channel_stddev_[channel_stddev_.size() == 1U ? 0U : channel]);
    auto * begin = output.data() + channel * pixels_;
    std::transform(begin, begin + pixels_, begin, [mean, stddev](const float value) {
      return (value - mean) / stddev;
    });
  }
}

bool EventTensorEncoderNode::accept_event_timestamp(const Timestamp event_us)
{
  if (last_event_us_ != 0 && event_us < last_event_us_) {
    const auto backward_us = static_cast<std::uint64_t>(last_event_us_ - event_us);
    out_of_order_events_.fetch_add(1, std::memory_order_relaxed);
    update_max(backward_jump_max_us_, backward_us);
    if (backward_us <= static_cast<std::uint64_t>(timestamp_backward_tolerance_us_)) {
      dropped_out_of_order_events_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    timestamp_reset_count_.fetch_add(1, std::memory_order_relaxed);
    reset_state(
      "event timestamp moved backwards by " + std::to_string(backward_us) +
      " us (tolerance=" + std::to_string(timestamp_backward_tolerance_us_) + " us)");
  }
  last_event_us_ = event_us;
  return true;
}

void EventTensorEncoderNode::reset_state(const std::string & reason)
{
  tensor_timing_->reset_epoch();
  window_events_.clear();
  cuda_pending_events_.clear();
  if (cuda_backend_ && cuda_stream_) {
    cuda_backend_->reset(*cuda_stream_);
  }
  std::fill(tensor_buffer_.begin(), tensor_buffer_.end(), 0.0F);
  next_publish_us_ = 0;
  publish_schedule_origin_us_ = 0;
  next_publish_target_us_ = 0;
  previous_window_end_us_ = 0;
  last_event_us_ = 0;
  has_last_event_sensor_update_ = false;
  in_flight_timestamp_ns_ = 0;
  last_snapshot_event_timestamp_us_ = 0;
  last_snapshot_header_timestamp_ns_ = 0;
  cuda_ring_dirty_ = false;
  inference_in_flight_ = false;
  cuda_events_since_snapshot_ = 0;
  queued_events_gauge_.store(0, std::memory_order_relaxed);
  RCLCPP_WARN(get_logger(), "Event tensor state reset: %s", reason.c_str());
}

void EventTensorEncoderNode::publish_diagnostics()
{
  std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
  const auto now_steady = std::chrono::steady_clock::now();
  const auto elapsed_s =
    std::chrono::duration<double>(now_steady - last_statistics_time_).count();
  last_statistics_time_ = now_steady;
  if (elapsed_s <= 0.0) {
    return;
  }

  const auto packets = received_packets_.exchange(0, std::memory_order_relaxed);
  const auto events = decoded_events_.exchange(0, std::memory_order_relaxed);
  const auto decode_calls = decode_calls_.exchange(0, std::memory_order_relaxed);
  const auto decode_ns = decode_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_max_ns = decode_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto packet_process_ns = packet_process_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto packet_process_max_ns =
    packet_process_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto tensors = published_tensors_.exchange(0, std::memory_order_relaxed);
  const auto full = full_windows_.exchange(0, std::memory_order_relaxed);
  const auto incremental = incremental_windows_.exchange(0, std::memory_order_relaxed);
  const auto represented = represented_windows_.exchange(0, std::memory_order_relaxed);
  const auto representation_ns = representation_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto representation_max_ns =
    representation_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto transfer_ns = transfer_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto transfer_max_ns = transfer_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto errors = decode_errors_.exchange(0, std::memory_order_relaxed);
  const auto publish_errors = publish_errors_.exchange(0, std::memory_order_relaxed);
  const auto feedback = inference_feedback_count_.exchange(0, std::memory_order_relaxed);
  const auto stale_feedback = stale_feedback_count_.exchange(0, std::memory_order_relaxed);
  const auto watchdog_timeouts = watchdog_timeout_count_.exchange(0, std::memory_order_relaxed);
  const auto fixed_rate_skipped =
    fixed_rate_skipped_windows_.exchange(0, std::memory_order_relaxed);
  const auto cuda_flushes = cuda_flush_count_.exchange(0, std::memory_order_relaxed);
  const auto cuda_events = cuda_flush_events_.exchange(0, std::memory_order_relaxed);
  const auto cuda_flush_ns = cuda_flush_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto cuda_flush_max_ns = cuda_flush_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto events_per_snapshot = events_per_snapshot_gauge_.load(std::memory_order_relaxed);
  const auto events_per_snapshot_max =
    events_per_snapshot_max_.exchange(0, std::memory_order_relaxed);
  const auto inference_ns = inference_round_trip_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto inference_max_ns =
    inference_round_trip_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto sensor_age_ns = output_sensor_age_ns_.exchange(0, std::memory_order_relaxed);
  const auto sensor_age_max_ns = output_sensor_age_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto sensor_age_samples =
    output_sensor_age_samples_.exchange(0, std::memory_order_relaxed);
  const auto out_of_bounds = out_of_bounds_events_.exchange(0, std::memory_order_relaxed);
  const auto out_of_order = out_of_order_events_.exchange(0, std::memory_order_relaxed);
  const auto dropped_out_of_order =
    dropped_out_of_order_events_.exchange(0, std::memory_order_relaxed);
  const auto timestamp_resets =
    timestamp_reset_count_.exchange(0, std::memory_order_relaxed);
  const auto backward_jump_max_us =
    backward_jump_max_us_.exchange(0, std::memory_order_relaxed);
  const auto empty = empty_windows_.exchange(0, std::memory_order_relaxed);
  const auto last_event_arrival_age_ms = has_last_event_arrival_ ?
    std::chrono::duration<double, std::milli>(now_steady - last_event_arrival_time_).count() :
    -1.0;

  if (dropped_out_of_order > 0) {
    RCLCPP_WARN(
      get_logger(),
      "Dropped %lu out-of-order events during the last %.3fs; "
      "maximum backward jump=%lu us, reset threshold=%ld us",
      static_cast<unsigned long>(dropped_out_of_order), elapsed_s,
      static_cast<unsigned long>(backward_jump_max_us), timestamp_backward_tolerance_us_);
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string("/event_tensor_encoder");
  status.hardware_id = "event_camera";
  status.level =
    errors == 0 && publish_errors == 0 && watchdog_timeouts == 0 &&
    fixed_rate_skipped == 0 && timestamp_resets == 0 ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message =
    errors == 0 && publish_errors == 0 && watchdog_timeouts == 0 &&
    fixed_rate_skipped == 0 && timestamp_resets == 0 ?
    "OK" : "event tensor deadline, timestamp, or processing issue detected";
  status.values = {
    diagnostic_number("packets_per_s", packets / elapsed_s),
    diagnostic_number("events_per_s", events / elapsed_s),
    diagnostic_number("tensors_per_s", tensors / elapsed_s),
    diagnostic_number("decode_ms_avg", decode_calls == 0 ? 0.0 : milliseconds(decode_ns) / decode_calls),
    diagnostic_number("decode_ms_max", milliseconds(decode_max_ns)),
    diagnostic_number(
      "packet_process_ms_avg", decode_calls == 0 ? 0.0 :
      milliseconds(packet_process_ns) / decode_calls),
    diagnostic_number("packet_process_ms_max", milliseconds(packet_process_max_ns)),
    diagnostic_number(
      "representation_ms_avg", represented == 0 ? 0.0 :
      milliseconds(representation_ns) / represented),
    diagnostic_number("representation_ms_max", milliseconds(representation_max_ns)),
    diagnostic_number(
      "transfer_enqueue_publish_ms_avg", tensors == 0 ? 0.0 :
      milliseconds(transfer_ns) / tensors),
    diagnostic_number("transfer_enqueue_publish_ms_max", milliseconds(transfer_max_ns)),
    diagnostic_number("full_windows", full),
    diagnostic_number("incremental_windows", incremental),
    diagnostic_number("empty_windows", empty),
    diagnostic_number("queued_events", queued_events_gauge_.load(std::memory_order_relaxed)),
    diagnostic_number("last_event_arrival_age_ms", last_event_arrival_age_ms),
    diagnostic_number("decode_errors", errors),
    diagnostic_number("publish_errors", publish_errors),
    diagnostic_number("out_of_bounds_events", out_of_bounds),
    diagnostic_number("out_of_order_events", out_of_order),
    diagnostic_number("dropped_out_of_order_events", dropped_out_of_order),
    diagnostic_number("timestamp_resets", timestamp_resets),
    diagnostic_number("max_backward_jump_us", backward_jump_max_us),
    diagnostic_number("timestamp_backward_tolerance_us", timestamp_backward_tolerance_us_),
    diagnostic_value("representation_backend", representation_backend_),
    diagnostic_value("inference_policy", inference_policy_),
    diagnostic_number("cuda_updates", cuda_flushes),
    diagnostic_number("cuda_events_transferred", cuda_events),
    diagnostic_number("events_per_snapshot", events_per_snapshot),
    diagnostic_number("events_per_snapshot_max", events_per_snapshot_max),
    diagnostic_number(
      "cuda_update_enqueue_ms_avg", cuda_flushes == 0 ? 0.0 :
      milliseconds(cuda_flush_ns) / cuda_flushes),
    diagnostic_number("cuda_update_enqueue_ms_max", milliseconds(cuda_flush_max_ns)),
    diagnostic_number("inference_feedback", feedback),
    diagnostic_number("stale_inference_feedback", stale_feedback),
    diagnostic_number("watchdog_timeouts", watchdog_timeouts),
    diagnostic_number("fixed_rate_skipped_windows", fixed_rate_skipped),
    diagnostic_number(
      "inference_round_trip_ms_avg", feedback == 0 ? 0.0 :
      milliseconds(inference_ns) / feedback),
    diagnostic_number("inference_round_trip_ms_max", milliseconds(inference_max_ns)),
    diagnostic_number(
      "output_sensor_age_ms_avg", sensor_age_samples == 0 ? 0.0 :
      milliseconds(sensor_age_ns) / sensor_age_samples),
    diagnostic_number("output_sensor_age_ms_max", milliseconds(sensor_age_max_ns)),
    diagnostic_value("inference_in_flight", inference_in_flight_ ? "true" : "false"),
    diagnostic_number("last_snapshot_sensor_timestamp_us", last_snapshot_event_timestamp_us_),
    diagnostic_number("bins", bins_),
    diagnostic_number("channels", channels_),
    diagnostic_number("window_ms", window_us_ / 1000.0),
    diagnostic_number("stride_ms", stride_us_ / 1000.0),
    diagnostic_number(
      "target_output_hz", output_rate_hz_ > 0.0 ? output_rate_hz_ : 1.0e6 / stride_us_),
    diagnostic_value("temporal_interpolation", temporal_interpolation_),
    diagnostic_value("polarity_mode", polarity_mode_),
    diagnostic_value("polarity_layout", polarity_layout_),
    diagnostic_value(
      "incremental_active",
      (use_incremental_ || (representation_backend_ == "cuda" && stride_us_ < window_us_)) ?
      "true" : "false"),
    diagnostic_value(
      "pinned_host_staging_active",
      representation_backend_ == "cpu" && std::all_of(
        staging_buffer_pinned_.cbegin(), staging_buffer_pinned_.cend(),
        [](const bool pinned) {return pinned;}) ? "true" : "false"),
    diagnostic_number(
      "incremental_shift_bins",
      representation_backend_ == "cuda" ?
      stride_us_ / (window_us_ / bins_) : static_cast<std::int64_t>(shift_bins_)),
    diagnostic_number(
      "incremental_reused_bins",
      representation_backend_ == "cuda" && stride_us_ < window_us_ ?
      bins_ - stride_us_ / (window_us_ / bins_) :
      (use_incremental_ ? static_cast<std::int64_t>(bins_) -
      static_cast<std::int64_t>(shift_bins_) : 0)),
    diagnostic_number("tensor_bytes", tensor_bytes_)
  };
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);

  debug_ = get_parameter("debug").as_bool();
  if (debug_) {
    RCLCPP_INFO(
      get_logger(),
      "event tensor: %.1f packet/s %.0f event/s %.1f tensor/s; "
      "packet avg/max %.3f/%.3fms, decode avg/max %.3f/%.3fms, "
      "repr avg/max %.3f/%.3fms, "
      "transfer-enqueue avg/max %.3f/%.3fms, full=%lu incremental=%lu queued=%lu",
      packets / elapsed_s, events / elapsed_s, tensors / elapsed_s,
      decode_calls == 0 ? 0.0 : milliseconds(packet_process_ns) / decode_calls,
      milliseconds(packet_process_max_ns),
      decode_calls == 0 ? 0.0 : milliseconds(decode_ns) / decode_calls,
      milliseconds(decode_max_ns),
      represented == 0 ? 0.0 : milliseconds(representation_ns) / represented,
      milliseconds(representation_max_ns),
      tensors == 0 ? 0.0 : milliseconds(transfer_ns) / tensors,
      milliseconds(transfer_max_ns), static_cast<unsigned long>(full),
      static_cast<unsigned long>(incremental),
      static_cast<unsigned long>(queued_events_gauge_.load(std::memory_order_relaxed)));
    if (representation_backend_ == "cuda") {
      RCLCPP_INFO(
        get_logger(),
        "event CUDA: updates=%lu events=%lu enqueue avg/max %.3f/%.3fms; "
        "fixed-rate skipped=%lu; TRT feedback=%lu round-trip avg/max %.3f/%.3fms "
        "watchdog=%lu stale=%lu",
        static_cast<unsigned long>(cuda_flushes), static_cast<unsigned long>(cuda_events),
        cuda_flushes == 0 ? 0.0 : milliseconds(cuda_flush_ns) / cuda_flushes,
        milliseconds(cuda_flush_max_ns), static_cast<unsigned long>(fixed_rate_skipped),
        static_cast<unsigned long>(feedback),
        feedback == 0 ? 0.0 : milliseconds(inference_ns) / feedback,
        milliseconds(inference_max_ns), static_cast<unsigned long>(watchdog_timeouts),
        static_cast<unsigned long>(stale_feedback));
    }
  }
}

}  // namespace openeb_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(openeb_ros2::EventTensorEncoderNode)
