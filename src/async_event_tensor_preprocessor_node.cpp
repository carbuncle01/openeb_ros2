#include "openeb_ros2/async_event_tensor_preprocessor_node.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#ifdef __linux__
#include <sched.h>
#endif

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "openeb_ros2/tensor_output.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"

namespace openeb_ros2
{
namespace
{

bool update_max(std::atomic<std::uint64_t> & maximum, const std::uint64_t value)
{
  auto current = maximum.load(std::memory_order_relaxed);
  while (
    current < value &&
    !maximum.compare_exchange_weak(current, value, std::memory_order_relaxed))
  {
  }
  return current < value;
}

diagnostic_msgs::msg::KeyValue value(std::string key, std::string data)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = std::move(key);
  output.value = std::move(data);
  return output;
}

template<typename T>
diagnostic_msgs::msg::KeyValue number(std::string key, const T data)
{
  return value(std::move(key), std::to_string(data));
}

double milliseconds(const std::uint64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1.0e6;
}

std::int64_t steady_nanoseconds(const std::chrono::steady_clock::time_point time)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

std::uint64_t thread_cpu_nanoseconds()
{
#ifdef CLOCK_THREAD_CPUTIME_ID
  timespec time{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time) == 0) {
    return static_cast<std::uint64_t>(time.tv_sec) * 1000000000ULL +
           static_cast<std::uint64_t>(time.tv_nsec);
  }
#endif
  return 0U;
}

int current_cpu()
{
#ifdef __linux__
  return sched_getcpu();
#else
  return -1;
#endif
}

std::int64_t read_cpu_frequency_khz(const int cpu, const char * file)
{
  if (cpu < 0) {
    return -1;
  }
  std::ifstream input(
    "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/cpufreq/" + file);
  std::int64_t frequency_khz{-1};
  if (input >> frequency_khz) {
    return frequency_khz;
  }
  return -1;
}

}  // namespace

AsyncEventTensorPreprocessorNode::AsyncEventTensorPreprocessorNode(
  const rclcpp::NodeOptions & options)
: Node("async_event_tensor_preprocessor", options)
{
  event_aggregation_ = declare_parameter<std::string>("event_aggregation", "events");
  if (event_aggregation_ != "events" && event_aggregation_ != "evt3_runs" &&
    event_aggregation_ != "evt3_runs_gpu") {
    throw std::invalid_argument("event_aggregation must be events, evt3_runs, or evt3_runs_gpu");
  }
  bins_ = declare_parameter<std::int64_t>("bins", 10);
  width_ = declare_parameter<std::int64_t>("width", 160);
  height_ = declare_parameter<std::int64_t>("height", 120);
  const auto window_ms = declare_parameter<double>("window_ms", 40.0);
  const auto stride_ms = declare_parameter<double>("stride_ms", 4.0);
  const auto output_rate_hz = declare_parameter<double>("output_rate_hz", 250.0);
  polarity_mode_ = declare_parameter<std::string>("polarity_mode", "separate");
  polarity_layout_ = declare_parameter<std::string>("polarity_layout", "polarity_major");
  const auto temporal_interpolation =
    declare_parameter<std::string>("temporal_interpolation", "none");
  tensor_name_ = declare_parameter<std::string>("tensor_name", "input_tensor");
  channel_mean_ = declare_parameter<std::vector<double>>(
    "channel_mean", std::vector<double>{0.0});
  channel_stddev_ = declare_parameter<std::vector<double>>(
    "channel_stddev", std::vector<double>{1.0});
  publish_empty_ = declare_parameter<bool>("publish_empty", true);
  debug_ = declare_parameter<bool>("debug", false);
  statistics_interval_s_ = declare_parameter<double>("statistics_interval_s", 1.0);
  deadline_ms_ = declare_parameter<double>("deadline_ms", 4.0);
  timestamp_backward_tolerance_us_ = declare_parameter<std::int64_t>(
    "timestamp_backward_tolerance_us", 4000);
  const auto transfer_events =
    declare_parameter<std::int64_t>("cuda_events_per_transfer", 8192);
  const auto gpu_chunk_events =
    declare_parameter<std::int64_t>("gpu_chunk_events", transfer_events);
  const auto packet_queue_capacity =
    declare_parameter<std::int64_t>("packet_queue_capacity", 64);
  const auto decoded_queue_capacity =
    declare_parameter<std::int64_t>("decoded_queue_capacity", 64);
  const auto decode_buffer_pool_capacity =
    declare_parameter<std::int64_t>("decode_buffer_pool_capacity", 4);
  const auto decode_buffer_pool_max_events =
    declare_parameter<std::int64_t>("decode_buffer_pool_max_events", 524288);
  max_queue_age_ms_ = declare_parameter<double>("max_queue_age_ms", 20.0);
  const auto subscription_depth = declare_parameter<std::int64_t>("subscription_depth", 16);
  const auto publisher_depth = declare_parameter<std::int64_t>("publisher_depth", 8);
  const auto diagnostics_topic = declare_parameter<std::string>(
    "diagnostics_topic", "tensor_diagnostics");

  if (
    bins_ <= 0 || width_ <= 0 || height_ <= 0 || !std::isfinite(window_ms) ||
    !std::isfinite(stride_ms) || !std::isfinite(output_rate_hz) ||
    window_ms <= 0.0 || stride_ms <= 0.0 || output_rate_hz <= 0.0 ||
    output_rate_hz > 1000.0)
  {
    throw std::invalid_argument("event tensor dimensions and timing must be positive");
  }
  if (polarity_mode_ != "signed" && polarity_mode_ != "separate") {
    throw std::invalid_argument("polarity_mode must be 'signed' or 'separate'");
  }
  if (polarity_layout_ != "polarity_major" && polarity_layout_ != "time_major") {
    throw std::invalid_argument("polarity_layout must be 'polarity_major' or 'time_major'");
  }
  if (temporal_interpolation != "none") {
    throw std::invalid_argument(
            "the asynchronous CUDA preprocessor currently supports temporal_interpolation=none");
  }
  if (
    transfer_events <= 0 || gpu_chunk_events <= 0 || packet_queue_capacity <= 0 ||
    decoded_queue_capacity <= 0 || subscription_depth <= 0 || publisher_depth <= 0 ||
    decode_buffer_pool_capacity <= 0 || decode_buffer_pool_max_events <= 0 ||
    timestamp_backward_tolerance_us_ < 0 ||
    !std::isfinite(statistics_interval_s_) || statistics_interval_s_ < 0.0 ||
    !std::isfinite(deadline_ms_) || deadline_ms_ <= 0.0 ||
    !std::isfinite(max_queue_age_ms_) || max_queue_age_ms_ <= 0.0)
  {
    throw std::invalid_argument("queue, CUDA, pool, diagnostics, and deadline values are invalid");
  }

  window_us_ = static_cast<std::int64_t>(std::llround(window_ms * 1000.0));
  stride_us_ = static_cast<std::int64_t>(std::llround(stride_ms * 1000.0));
  output_period_us_ = static_cast<std::int64_t>(std::llround(1.0e6 / output_rate_hz));
  if (window_us_ % bins_ != 0) {
    throw std::invalid_argument("window_us must be divisible by bins");
  }
  const auto bin_width_us = window_us_ / bins_;
  if (stride_us_ % bin_width_us != 0) {
    throw std::invalid_argument(
            "stride must be an integer multiple of window_us/bins");
  }
  if (output_period_us_ < stride_us_) {
    throw std::invalid_argument("output_rate_hz cannot exceed the event representation stride rate");
  }

  packet_queue_capacity_ = static_cast<std::size_t>(packet_queue_capacity);
  decoded_queue_capacity_ = static_cast<std::size_t>(decoded_queue_capacity);
  gpu_chunk_events_ = static_cast<std::size_t>(gpu_chunk_events);
  decode_buffer_initial_events_ = static_cast<std::size_t>(transfer_events) * 2U;
  decode_buffer_pool_capacity_ = static_cast<std::size_t>(decode_buffer_pool_capacity);
  decode_buffer_pool_max_events_ = static_cast<std::size_t>(decode_buffer_pool_max_events);
  if (decode_buffer_pool_max_events_ < decode_buffer_initial_events_) {
    throw std::invalid_argument(
            "decode_buffer_pool_max_events must be at least twice cuda_events_per_transfer");
  }
  channels_ = static_cast<std::size_t>(bins_) * (polarity_mode_ == "separate" ? 2U : 1U);
  const auto pixels = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  if (pixels > std::numeric_limits<std::size_t>::max() / channels_) {
    throw std::overflow_error("event tensor dimensions overflow size_t");
  }
  tensor_bytes_ = channels_ * pixels * sizeof(float);
  const auto validate_channels = [this](const std::vector<double> & values, const bool positive) {
      if (values.size() != 1U && values.size() != channels_) {
        throw std::invalid_argument("normalization must have one value or one value per channel");
      }
      for (const auto item : values) {
        if (!std::isfinite(item) || (positive && item <= 0.0)) {
          throw std::invalid_argument("normalization contains an invalid value");
        }
      }
    };
  validate_channels(channel_mean_, false);
  validate_channels(channel_stddev_, true);

  cuda_stream_ = nvidia::isaac_ros::common::createCudaStream(
    "AsyncEventTensorPreprocessorNode");
  if (event_aggregation_ == "events") {
    cuda_backend_ = std::make_unique<EventTensorCudaBackend>(
      static_cast<std::size_t>(width_), static_cast<std::size_t>(height_),
      static_cast<std::size_t>(bins_), polarity_mode_ == "separate",
      polarity_layout_ == "polarity_major", window_us_,
      static_cast<std::size_t>(transfer_events), channel_mean_, channel_stddev_);
  } else {
    direct_histogram_ = std::make_unique<Evt3RunsHistogram>(
      width_, height_, bins_, window_us_, polarity_mode_ == "separate",
      polarity_layout_ == "polarity_major", timestamp_backward_tolerance_us_);
    if (event_aggregation_ == "evt3_runs_gpu") {
      cuda_backend_ = std::make_unique<EventTensorCudaBackend>(
        width_, height_, bins_, polarity_mode_ == "separate",
        polarity_layout_ == "polarity_major", window_us_,
        static_cast<std::size_t>(transfer_events), channel_mean_, channel_stddev_, false);
    } else {
      direct_snapshot_buffer_.resize(tensor_bytes_ / sizeof(float));
    }
  }
  decoder_factory_ = std::make_unique<DecoderFactory>();

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  rclcpp::PublisherOptions publisher_options;
  publisher_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  if (declare_parameter<bool>("subscribe_packets", true)) {
    event_subscription_ = create_subscription<EventPacket>(
      "events",
      rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(subscription_depth)))
      .best_effort().durability_volatile(),
      std::bind(&AsyncEventTensorPreprocessorNode::on_packet, this, std::placeholders::_1),
      subscription_options);
  }
  tensor_timing_ = std::make_unique<TensorTiming>(*this);
  tensor_publisher_ = create_publisher<TensorList>(
    "tensor",
    rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(publisher_depth)))
    .reliable().durability_volatile(), publisher_options);
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);

  last_statistics_time_ = std::chrono::steady_clock::now();
  if (statistics_interval_s_ > 0.0) {
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max<std::int64_t>(
          1, static_cast<std::int64_t>(statistics_interval_s_ * 1000.0))),
      std::bind(&AsyncEventTensorPreprocessorNode::publish_diagnostics, this));
  }

  decode_thread_ = std::thread(&AsyncEventTensorPreprocessorNode::decode_loop, this);
  gpu_thread_ = std::thread(&AsyncEventTensorPreprocessorNode::gpu_loop, this);
  RCLCPP_INFO(
    get_logger(),
    "Asynchronous event tensor preprocessor: shape=[1,%zu,%ld,%ld], window=%.3fms, "
    "stride=%.3fms, output=%.1fHz, deadline=%.3fms, packet_queue=%zu, decoded_queue=%zu, "
    "gpu_chunk=%zu, decode_buffer_pool=%zu x <=%zu events",
    channels_, height_, width_, window_ms, stride_ms, output_rate_hz, deadline_ms_,
    packet_queue_capacity_, decoded_queue_capacity_, gpu_chunk_events_,
    decode_buffer_pool_capacity_, decode_buffer_pool_max_events_);
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

AsyncEventTensorPreprocessorNode::~AsyncEventTensorPreprocessorNode()
{
  stopping_.store(true, std::memory_order_release);
  packet_cv_.notify_all();
  decoded_cv_.notify_all();
  if (decode_thread_.joinable()) {
    decode_thread_.join();
  }
  if (gpu_thread_.joinable()) {
    gpu_thread_.join();
  }
  if (cuda_stream_) {
    (void)cudaStreamSynchronize(*cuda_stream_);
  }
  cuda_backend_.reset();
}

void AsyncEventTensorPreprocessorNode::on_packet(EventPacket::UniquePtr packet)
{
  const auto started = std::chrono::steady_clock::now();
  if (!packet || packet->events.empty()) {
    return;
  }
  const auto packet_sequence = static_cast<std::uint64_t>(packet->seq);
  received_packets_.fetch_add(1, std::memory_order_relaxed);
  received_packets_total_.fetch_add(1, std::memory_order_relaxed);
  const bool had_previous_sequence =
    received_sequence_initialized_.exchange(true, std::memory_order_relaxed);
  const auto previous_sequence =
    last_received_sequence_.exchange(packet_sequence, std::memory_order_relaxed);
  if (had_previous_sequence) {
    if (packet_sequence > previous_sequence + 1U) {
      const auto missing = packet_sequence - previous_sequence - 1U;
      input_sequence_gap_occurrences_.fetch_add(1, std::memory_order_relaxed);
      input_sequence_missing_packets_.fetch_add(missing, std::memory_order_relaxed);
      update_max(input_sequence_gap_max_, missing);
    } else if (packet_sequence <= previous_sequence) {
      input_sequence_reorders_.fetch_add(1, std::memory_order_relaxed);
    }
  }
  const auto arrival_ns = steady_nanoseconds(started);
  const auto previous_arrival_ns =
    last_packet_arrival_steady_ns_.exchange(arrival_ns, std::memory_order_relaxed);
  if (previous_arrival_ns > 0 && arrival_ns > previous_arrival_ns) {
    const auto interarrival_ns = static_cast<std::uint64_t>(arrival_ns - previous_arrival_ns);
    packet_interarrival_samples_.fetch_add(1, std::memory_order_relaxed);
    packet_interarrival_ns_.fetch_add(interarrival_ns, std::memory_order_relaxed);
    update_max(packet_interarrival_max_ns_, interarrival_ns);
    if (interarrival_ns > static_cast<std::uint64_t>(output_period_us_) * 1000U) {
      packet_interarrival_over_output_period_.fetch_add(1, std::memory_order_relaxed);
    }
  }
  {
    std::lock_guard<std::mutex> lock(packet_mutex_);
    if (packet_queue_.size() >= packet_queue_capacity_) {
      auto & dropped = packet_queue_.front();
      dropped_packet_queue_bytes_.fetch_add(
        dropped.packet ? dropped.packet->events.size() : 0U, std::memory_order_relaxed);
      dropped_packet_queue_packets_.fetch_add(1, std::memory_order_relaxed);
      packet_queue_.pop_front();
    }
    packet_queue_.push_back(PacketWork{std::move(packet), started});
    packet_queue_depth_.store(packet_queue_.size(), std::memory_order_relaxed);
    update_max(packet_queue_depth_max_, packet_queue_.size());
  }
  enqueued_packets_.fetch_add(1, std::memory_order_relaxed);
  packet_cv_.notify_one();
  const auto elapsed = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - started).count());
  packet_callback_time_ns_.fetch_add(elapsed, std::memory_order_relaxed);
  update_max(packet_callback_time_max_ns_, elapsed);
}

void AsyncEventTensorPreprocessorNode::decode_loop()
{
  while (!stopping_.load(std::memory_order_acquire)) {
    PacketWork work;
    {
      std::unique_lock<std::mutex> lock(packet_mutex_);
      packet_cv_.wait(lock, [this]() {
          return stopping_.load(std::memory_order_acquire) || !packet_queue_.empty();
        });
      if (stopping_.load(std::memory_order_acquire)) {
        break;
      }
      work = std::move(packet_queue_.front());
      packet_queue_.pop_front();
      packet_queue_depth_.store(packet_queue_.size(), std::memory_order_relaxed);
    }
    decode_packet(std::move(work));
  }
}

std::unique_ptr<std::vector<CudaEvent>>
AsyncEventTensorPreprocessorNode::acquire_decode_buffer()
{
  decode_buffer_pool_acquires_.fetch_add(1, std::memory_order_relaxed);
  std::unique_ptr<std::vector<CudaEvent>> buffer;
  {
    std::lock_guard<std::mutex> lock(decode_buffer_pool_mutex_);
    if (!decode_buffer_pool_.empty()) {
      const auto largest = std::max_element(
        decode_buffer_pool_.begin(), decode_buffer_pool_.end(),
        [](const auto & left, const auto & right) {
          return left->capacity() < right->capacity();
        });
      buffer = std::move(*largest);
      decode_buffer_pool_.erase(largest);
      decode_buffer_pool_depth_.store(decode_buffer_pool_.size(), std::memory_order_relaxed);
      decode_buffer_pool_retained_events_.fetch_sub(
        buffer->capacity(), std::memory_order_relaxed);
    }
  }
  if (buffer) {
    decode_buffer_pool_hits_.fetch_add(1, std::memory_order_relaxed);
  } else {
    decode_buffer_pool_misses_.fetch_add(1, std::memory_order_relaxed);
    buffer = std::make_unique<std::vector<CudaEvent>>();
  }
  buffer->clear();
  if (buffer->capacity() < decode_buffer_initial_events_) {
    buffer->reserve(decode_buffer_initial_events_);
  }
  return buffer;
}

void AsyncEventTensorPreprocessorNode::release_decode_buffer(
  std::unique_ptr<std::vector<CudaEvent>> buffer)
{
  if (!buffer) {
    return;
  }
  decode_buffer_pool_returns_.fetch_add(1, std::memory_order_relaxed);
  buffer->clear();
  if (buffer->capacity() > decode_buffer_pool_max_events_) {
    decode_buffer_pool_discarded_oversize_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  std::unique_ptr<std::vector<CudaEvent>> discarded;
  {
    std::lock_guard<std::mutex> lock(decode_buffer_pool_mutex_);
    if (decode_buffer_pool_.size() < decode_buffer_pool_capacity_) {
      decode_buffer_pool_retained_events_.fetch_add(
        buffer->capacity(), std::memory_order_relaxed);
      decode_buffer_pool_.push_back(std::move(buffer));
    } else {
      const auto smallest = std::min_element(
        decode_buffer_pool_.begin(), decode_buffer_pool_.end(),
        [](const auto & left, const auto & right) {
          return left->capacity() < right->capacity();
        });
      if ((*smallest)->capacity() < buffer->capacity()) {
        decode_buffer_pool_retained_events_.fetch_sub(
          (*smallest)->capacity(), std::memory_order_relaxed);
        decode_buffer_pool_retained_events_.fetch_add(
          buffer->capacity(), std::memory_order_relaxed);
        discarded = std::move(*smallest);
        *smallest = std::move(buffer);
      } else {
        discarded = std::move(buffer);
      }
    }
    decode_buffer_pool_depth_.store(decode_buffer_pool_.size(), std::memory_order_relaxed);
    update_max(decode_buffer_pool_depth_max_, decode_buffer_pool_.size());
  }
  if (discarded) {
    decode_buffer_pool_discarded_full_.fetch_add(1, std::memory_order_relaxed);
  }
}

void AsyncEventTensorPreprocessorNode::decode_packet(PacketWork work)
{
  const auto started = std::chrono::steady_clock::now();
  const auto thread_cpu_started_ns = thread_cpu_nanoseconds();
  const auto cpu_started = current_cpu();
  const auto record_service = [this, started, thread_cpu_started_ns, cpu_started]() {
      const auto finished = std::chrono::steady_clock::now();
      const auto service_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(finished - started).count());
      const auto thread_cpu_finished_ns = thread_cpu_nanoseconds();
      const auto thread_cpu_ns = thread_cpu_finished_ns >= thread_cpu_started_ns ?
        thread_cpu_finished_ns - thread_cpu_started_ns : 0U;
      const auto scheduling_delay_ns = service_ns > thread_cpu_ns ? service_ns - thread_cpu_ns : 0U;
      const auto cpu_finished = current_cpu();
      decode_service_calls_.fetch_add(1, std::memory_order_relaxed);
      decode_service_time_ns_.fetch_add(service_ns, std::memory_order_relaxed);
      update_max(decode_service_time_max_ns_, service_ns);
      decode_thread_cpu_time_ns_.fetch_add(thread_cpu_ns, std::memory_order_relaxed);
      update_max(decode_thread_cpu_time_max_ns_, thread_cpu_ns);
      decode_scheduling_delay_ns_.fetch_add(scheduling_delay_ns, std::memory_order_relaxed);
      update_max(decode_scheduling_delay_max_ns_, scheduling_delay_ns);
      if (cpu_started >= 0 && cpu_finished >= 0 && cpu_started != cpu_finished) {
        decode_cpu_migrations_.fetch_add(1, std::memory_order_relaxed);
      }
      decode_last_cpu_.store(cpu_finished, std::memory_order_relaxed);
    };
  const auto wait_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      started - work.enqueued_at).count());
  decode_queue_wait_ns_.fetch_add(wait_ns, std::memory_order_relaxed);
  update_max(decode_queue_wait_max_ns_, wait_ns);

  auto & packet = work.packet;
  if (!packet) {
    record_service();
    return;
  }
  if (milliseconds(wait_ns) > max_queue_age_ms_) {
    stale_packet_queue_packets_.fetch_add(1, std::memory_order_relaxed);
    dropped_packet_queue_bytes_.fetch_add(packet->events.size(), std::memory_order_relaxed);
    record_service();
    return;
  }
  if (direct_histogram_) {
    decode_direct_packet(std::move(work));
    record_service();
    return;
  }
  decode_scratch_ = acquire_decode_buffer();
  decode_scratch_->clear();
  const auto decode_capacity_before =
    static_cast<std::uint64_t>(decode_scratch_->capacity());
  decode_buffer_capacity_before_events_.store(
    decode_capacity_before, std::memory_order_relaxed);
  decode_reset_pending_ = false;
  if (packet_width_ != packet->width || packet_height_ != packet->height) {
    if (packet_width_ != 0U || packet_height_ != 0U) {
      request_decode_reset();
    }
    rebuild_coordinate_luts(packet->width, packet->height);
  }
  try {
    auto * decoder = decoder_factory_->getInstance(*packet);
    if (decoder == nullptr) {
      throw std::runtime_error("no decoder for event packet encoding '" + packet->encoding + "'");
    }
    while (decoder->decode(*packet, this)) {
    }
  } catch (const std::exception & error) {
    decode_errors_.fetch_add(1, std::memory_order_relaxed);
    decoder_factory_ = std::make_unique<DecoderFactory>();
    request_decode_reset();
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Asynchronous event decode failed: %s", error.what());
  }

  const auto finished_at = std::chrono::steady_clock::now();
  const auto elapsed_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(finished_at - started).count());
  const auto packet_events = static_cast<std::uint64_t>(decode_scratch_->size());
  const auto decode_capacity_after =
    static_cast<std::uint64_t>(decode_scratch_->capacity());
  decoded_packets_.fetch_add(1, std::memory_order_relaxed);
  decoded_events_.fetch_add(packet_events, std::memory_order_relaxed);
  decode_time_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed);
  if (update_max(decode_time_max_ns_, elapsed_ns)) {
    decode_events_at_time_max_.store(packet_events, std::memory_order_relaxed);
  }
  if (update_max(decode_packet_events_max_, packet_events)) {
    decode_time_at_events_max_ns_.store(elapsed_ns, std::memory_order_relaxed);
  }
  decode_correlation_samples_.fetch_add(1, std::memory_order_relaxed);
  decode_correlation_events_sum_.fetch_add(packet_events, std::memory_order_relaxed);
  decode_correlation_time_ns_sum_.fetch_add(elapsed_ns, std::memory_order_relaxed);
  decode_correlation_events_squared_sum_.fetch_add(
    packet_events * packet_events, std::memory_order_relaxed);
  decode_correlation_time_ns_squared_sum_.fetch_add(
    elapsed_ns * elapsed_ns, std::memory_order_relaxed);
  decode_correlation_cross_sum_.fetch_add(
    packet_events * elapsed_ns, std::memory_order_relaxed);
  update_max(decode_buffer_capacity_after_events_max_, decode_capacity_after);
  if (decode_capacity_after > decode_capacity_before) {
    const auto growth = decode_capacity_after - decode_capacity_before;
    decode_buffer_growth_packets_.fetch_add(1, std::memory_order_relaxed);
    decode_buffer_growth_events_.fetch_add(growth, std::memory_order_relaxed);
    update_max(decode_buffer_growth_events_max_, growth);
  }

  if (decode_scratch_->empty() && !decode_reset_pending_) {
    release_decode_buffer(std::move(decode_scratch_));
    record_service();
    return;
  }
  const auto handoff_started = std::chrono::steady_clock::now();
  auto events = std::move(decode_scratch_);
  const auto packet_stamp_ns =
    static_cast<std::int64_t>(packet->header.stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(packet->header.stamp.nanosec);
  const auto sensor_to_ros_offset_ns = events->empty() ? 0LL :
    packet_stamp_ns - events->back().timestamp_us * 1000LL;
  enqueue_decoded(DecodedWork{
      std::move(events), 0U, packet->header.frame_id, sensor_to_ros_offset_ns,
      work.enqueued_at, finished_at, decode_reset_pending_});
  const auto handoff_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - handoff_started).count());
  decode_handoff_time_ns_.fetch_add(handoff_ns, std::memory_order_relaxed);
  update_max(decode_handoff_time_max_ns_, handoff_ns);
  record_service();
}

void AsyncEventTensorPreprocessorNode::enqueue_decoded(DecodedWork work)
{
  std::lock_guard<std::mutex> lock(decoded_mutex_);
  if (decoded_queue_.size() >= decoded_queue_capacity_) {
    auto dropped = std::move(decoded_queue_.front());
    decoded_queue_.pop_front();
    const auto remaining = dropped.events && dropped.offset < dropped.events->size() ?
      dropped.events->size() - dropped.offset : 0U;
    dropped_decoded_batches_.fetch_add(1, std::memory_order_relaxed);
    dropped_decoded_events_.fetch_add(remaining, std::memory_order_relaxed);
    release_decode_buffer(std::move(dropped.events));
    // A reset marker must survive queue overflow or two timestamp epochs could be mixed.
    if (dropped.reset_before) {
      work.reset_before = true;
      for (auto & queued : decoded_queue_) {
        const auto queued_remaining =
          queued.events && queued.offset < queued.events->size() ?
          queued.events->size() - queued.offset : 0U;
        dropped_decoded_batches_.fetch_add(1, std::memory_order_relaxed);
        dropped_decoded_events_.fetch_add(queued_remaining, std::memory_order_relaxed);
        release_decode_buffer(std::move(queued.events));
      }
      decoded_queue_.clear();
    }
  }
  decoded_queue_.push_back(std::move(work));
  decoded_queue_depth_.store(decoded_queue_.size(), std::memory_order_relaxed);
  update_max(decoded_queue_depth_max_, decoded_queue_.size());
  decoded_cv_.notify_one();
}

bool AsyncEventTensorPreprocessorNode::pop_decoded(DecodedWork & work)
{
  std::lock_guard<std::mutex> lock(decoded_mutex_);
  if (decoded_queue_.empty()) {
    return false;
  }
  work = std::move(decoded_queue_.front());
  decoded_queue_.pop_front();
  decoded_queue_depth_.store(decoded_queue_.size(), std::memory_order_relaxed);
  return true;
}

void AsyncEventTensorPreprocessorNode::decode_direct_packet(PacketWork work)
{
  const auto started = std::chrono::steady_clock::now();
  const auto & packet = *work.packet;
  std::lock_guard<std::mutex> lock(direct_mutex_);
  const auto before = direct_histogram_->statistics();
  try {
    if (packet.encoding != "evt3") throw std::invalid_argument("evt3_runs requires evt3 encoding");
    if (packet.width != packet_width_ || packet.height != packet_height_) {
      direct_histogram_->set_geometry(packet.width, packet.height);
      packet_width_ = packet.width; packet_height_ = packet.height;
      coordinate_lut_rebuilds_.fetch_add(1, std::memory_order_relaxed);
    } else if (direct_has_sequence_ && static_cast<std::uint64_t>(packet.seq) != direct_sequence_+1) {
      // Lost compressed words invalidate y/time/vector state. Never mix epochs.
      direct_histogram_->reset_stream();
    }
    direct_sequence_ = packet.seq; direct_has_sequence_ = true;
    direct_histogram_->feed(packet.events.data(), packet.events.size());
    direct_frame_id_ = packet.header.frame_id;
    if (direct_histogram_->ready() && direct_histogram_->statistics().accepted > before.accepted) {
      const auto stamp = static_cast<std::int64_t>(packet.header.stamp.sec)*1000000000LL +
        packet.header.stamp.nanosec;
      direct_offset_ns_ = stamp - direct_histogram_->latest()*1000LL;
    }
  } catch (const std::exception & error) {
    direct_histogram_->reset_stream();
    decode_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "Direct EVT3 aggregation failed: %s", error.what());
  }
  const auto after = direct_histogram_->statistics();
  const auto count = after.accepted-before.accepted;
  out_of_bounds_events_.fetch_add(after.bounds-before.bounds, std::memory_order_relaxed);
  out_of_order_events_.fetch_add(after.backward-before.backward, std::memory_order_relaxed);
  timestamp_resets_.fetch_add(after.resets-before.resets, std::memory_order_relaxed);
  dropped_out_of_order_events_.fetch_add(after.dropped-before.dropped, std::memory_order_relaxed);
  update_max(backward_jump_max_us_, after.backward_max_us);
  decoded_events_.fetch_add(count, std::memory_order_relaxed);
  decoded_packets_.fetch_add(1, std::memory_order_relaxed);
  const auto elapsed = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now()-started).count());
  decode_time_ns_.fetch_add(elapsed, std::memory_order_relaxed);
  if (update_max(decode_time_max_ns_, elapsed)) {
    decode_events_at_time_max_.store(count, std::memory_order_relaxed);
  }
  if (update_max(decode_packet_events_max_, count)) {
    decode_time_at_events_max_ns_.store(elapsed, std::memory_order_relaxed);
  }
  direct_wakeup_.store(true, std::memory_order_release);
  decoded_cv_.notify_one();
}

void AsyncEventTensorPreprocessorNode::direct_gpu_loop()
{
  const auto period = std::chrono::microseconds(output_period_us_);
  while (!stopping_.load(std::memory_order_acquire)) {
    {
      std::lock_guard<std::mutex> lock(direct_mutex_);
      const auto epoch = direct_histogram_->epoch();
      if (epoch != direct_snapshot_epoch_) {
        if (direct_snapshot_epoch_ != 0) tensor_timing_->reset_epoch();
        direct_snapshot_epoch_ = epoch;
        next_window_end_us_ = 0; publish_schedule_origin_us_ = 0;
        next_window_target_us_ = 0; last_header_timestamp_ns_ = 0;
      }
      if (next_window_end_us_ == 0 && direct_histogram_->ready()) {
        const auto ready_end = direct_histogram_->latest_window_end();
        if (ready_end > 0) {
          publish_schedule_origin_us_ = ready_end;
          const auto estimated = (now().nanoseconds()-direct_offset_ns_)/1000LL;
          const auto target = std::max<std::int64_t>(ready_end, estimated);
          next_window_target_us_ = ready_end + ((target-ready_end)/stride_us_)*stride_us_;
          next_window_end_us_ = aligned_window_end(next_window_target_us_);
          next_snapshot_at_ = std::chrono::steady_clock::now();
        }
      }
    }
    if (next_window_end_us_ > 0) {
      const auto current = std::chrono::steady_clock::now();
      if (current >= next_snapshot_at_) {
        const auto late = std::chrono::duration_cast<std::chrono::microseconds>(current-next_snapshot_at_).count();
        const auto due = late/output_period_us_+1;
        if (due > 1) {
          skipped_windows_.fetch_add(due-1, std::memory_order_relaxed);
          deadline_misses_.fetch_add(due-1, std::memory_order_relaxed);
        }
        publish_snapshot(aligned_window_end(next_window_target_us_+(due-1)*output_period_us_),
          next_snapshot_at_+period*(due-1));
        next_window_target_us_ += due*output_period_us_;
        next_window_end_us_ = aligned_window_end(next_window_target_us_);
        next_snapshot_at_ += period*due;
      }
    }
    std::unique_lock<std::mutex> lock(decoded_mutex_);
    const auto wake = [this]() {
        return stopping_.load(std::memory_order_acquire) ||
               direct_wakeup_.exchange(false, std::memory_order_acq_rel);
      };
    if (next_window_end_us_ > 0) decoded_cv_.wait_until(lock, next_snapshot_at_, wake);
    else decoded_cv_.wait_for(lock, period, wake);
  }
}

void AsyncEventTensorPreprocessorNode::gpu_loop()
{
  if (direct_histogram_) {direct_gpu_loop(); return;}
  DecodedWork active;
  bool has_active = false;
  const auto period = std::chrono::microseconds(output_period_us_);
  while (!stopping_.load(std::memory_order_acquire)) {
    auto now_steady = std::chrono::steady_clock::now();
    if (next_window_end_us_ > 0 && now_steady >= next_snapshot_at_) {
      const auto late_us = std::chrono::duration_cast<std::chrono::microseconds>(
        now_steady - next_snapshot_at_).count();
      const auto due = static_cast<std::uint64_t>(late_us / output_period_us_) + 1U;
      if (due > 1U) {
        skipped_windows_.fetch_add(due - 1U, std::memory_order_relaxed);
        deadline_misses_.fetch_add(due - 1U, std::memory_order_relaxed);
      }
      const auto publish_target = next_window_target_us_ +
        static_cast<std::int64_t>(due - 1U) * output_period_us_;
      const auto publish_end = aligned_window_end(publish_target);
      const auto scheduled_at = next_snapshot_at_ + period * static_cast<std::int64_t>(due - 1U);
      publish_snapshot(publish_end, scheduled_at);
      next_window_target_us_ += static_cast<std::int64_t>(due) * output_period_us_;
      next_window_end_us_ = aligned_window_end(next_window_target_us_);
      next_snapshot_at_ += period * static_cast<std::int64_t>(due);
      continue;
    }

    if (!has_active) {
      has_active = pop_decoded(active);
    }
    if (has_active) {
      apply_chunk(active);
      has_active = active.events && active.offset < active.events->size();
      if (!has_active) {
        release_decode_buffer(std::move(active.events));
      }
      continue;
    }

    std::unique_lock<std::mutex> lock(decoded_mutex_);
    if (next_window_end_us_ > 0) {
      decoded_cv_.wait_until(lock, next_snapshot_at_, [this]() {
          return stopping_.load(std::memory_order_acquire) || !decoded_queue_.empty();
        });
    } else {
      decoded_cv_.wait(lock, [this]() {
          return stopping_.load(std::memory_order_acquire) || !decoded_queue_.empty();
        });
    }
  }
}

void AsyncEventTensorPreprocessorNode::apply_chunk(DecodedWork & work)
{
  if (work.reset_before) {
    tensor_timing_->reset_epoch();
    cuda_backend_->reset(*cuda_stream_);
    publish_schedule_origin_us_ = 0;
    next_window_target_us_ = 0;
    next_window_end_us_ = 0;
    last_header_timestamp_ns_ = 0;
    snapshot_event_version_ = applied_event_version_;
    work.reset_before = false;
  }
  if (!work.events || work.offset >= work.events->size()) {
    return;
  }
  const auto queue_age_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - work.decoded_at).count());
  if (milliseconds(queue_age_ns) > max_queue_age_ms_) {
    const auto remaining = work.events->size() - work.offset;
    stale_decoded_batches_.fetch_add(1, std::memory_order_relaxed);
    stale_decoded_events_.fetch_add(remaining, std::memory_order_relaxed);
    work.offset = work.events->size();
    return;
  }
  const auto count = std::min(gpu_chunk_events_, work.events->size() - work.offset);
  const auto started = std::chrono::steady_clock::now();
  try {
    cuda_backend_->update(work.events->data() + work.offset, count, *cuda_stream_);
  } catch (const std::exception & error) {
    publish_errors_.fetch_add(1, std::memory_order_relaxed);
    work.offset = work.events->size();
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Asynchronous CUDA event update failed: %s", error.what());
    return;
  }
  const auto finished_at = std::chrono::steady_clock::now();
  const auto elapsed_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(finished_at - started).count());
  const auto queue_wait_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(started - work.decoded_at).count());
  gpu_updates_.fetch_add(1, std::memory_order_relaxed);
  gpu_events_.fetch_add(count, std::memory_order_relaxed);
  gpu_update_time_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed);
  update_max(gpu_update_time_max_ns_, elapsed_ns);
  gpu_queue_wait_ns_.fetch_add(queue_wait_ns, std::memory_order_relaxed);
  update_max(gpu_queue_wait_max_ns_, queue_wait_ns);
  work.offset += count;
  applied_event_version_ += count;
  events_since_snapshot_ += count;
  frame_id_ = work.frame_id;
  sensor_to_ros_offset_ns_ = work.sensor_to_ros_offset_ns;
  const auto newest_sensor_us = (*work.events)[work.offset - 1U].timestamp_us;
  last_applied_event_ros_ns_.store(
    ros_timestamp_ns(newest_sensor_us), std::memory_order_relaxed);

  if (next_window_end_us_ == 0) {
    const auto ready_end = cuda_backend_->latest_window_end_us();
    if (ready_end > 0) {
      publish_schedule_origin_us_ = ready_end;
      // Anchor the fixed-rate schedule to sensor "now", not to the first GPU
      // chunk.  The first chunk may be several milliseconds old while startup
      // queues are drained; retaining that offset would bias every downstream
      // sensor-to-command latency in the run by a constant amount.
      const auto current_ros_ns = now().nanoseconds();
      const std::int64_t estimated_sensor_now_us =
        current_ros_ns > sensor_to_ros_offset_ns_ ?
        (current_ros_ns - sensor_to_ros_offset_ns_) / std::int64_t{1000} :
        static_cast<std::int64_t>(ready_end);
      const auto target_us = std::max<std::int64_t>(
        static_cast<std::int64_t>(ready_end), estimated_sensor_now_us);
      const auto target_offset_us = target_us - publish_schedule_origin_us_;
      const auto stride_steps = target_offset_us / stride_us_;
      next_window_target_us_ =
        publish_schedule_origin_us_ + stride_steps * stride_us_;
      next_window_end_us_ = next_window_target_us_;
      const auto initial_lag_us = std::max<std::int64_t>(
        0, estimated_sensor_now_us - next_window_end_us_);
      schedule_initial_lag_ns_.store(
        static_cast<std::uint64_t>(initial_lag_us) * 1000U,
        std::memory_order_relaxed);
      next_snapshot_at_ = finished_at;
    }
  }
}

void AsyncEventTensorPreprocessorNode::publish_snapshot(
  const std::int64_t window_end_us, const SteadyTime scheduled_at)
{
  if (!direct_histogram_ && !cuda_backend_->ready()) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const auto started_ros_ns = now().nanoseconds();
  const auto lateness_ns = started > scheduled_at ? static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(started - scheduled_at).count()) : 0U;
  snapshot_lateness_ns_.fetch_add(lateness_ns, std::memory_order_relaxed);
  update_max(snapshot_lateness_max_ns_, lateness_ns);
  try {
    tensor_msgs::msg::ExperimentalTensor tensor;
    std::vector<std::int64_t> shape{
      1, static_cast<std::int32_t>(channels_), static_cast<std::int32_t>(height_),
      static_cast<std::int32_t>(width_)};
    {
      auto write_handle = allocate_tensor(tensor, shape, *cuda_stream_);
      if (direct_histogram_) {
        {
          std::lock_guard<std::mutex> lock(direct_mutex_);
          if (!direct_histogram_->ready() || direct_histogram_->epoch() != direct_snapshot_epoch_) {
            return;  // The publisher worker will establish the new epoch schedule.
          }
          if (event_aggregation_ == "evt3_runs_gpu") {
            direct_histogram_->take_dirty_bins(direct_bin_updates_);
          } else {
            direct_histogram_->snapshot(direct_snapshot_buffer_.data(), window_end_us,
              channel_mean_, channel_stddev_);
          }
          frame_id_ = direct_frame_id_;
          sensor_to_ros_offset_ns_ = direct_offset_ns_;
          last_applied_event_ros_ns_.store(
            direct_histogram_->latest()*1000LL + direct_offset_ns_, std::memory_order_relaxed);
          applied_event_version_ = direct_histogram_->statistics().accepted;
          events_since_snapshot_ = applied_event_version_ - snapshot_event_version_;
        }
        if (event_aggregation_ == "evt3_runs_gpu") {
          cuda_backend_->update_bins(direct_bin_updates_, *cuda_stream_);
          cuda_backend_->snapshot(reinterpret_cast<float *>(write_handle.get_ptr()),
            window_end_us, *cuda_stream_);
        } else {
          // The CPU-only aggregation mode still uploads a complete snapshot.
          auto result = cudaMemcpyAsync(write_handle.get_ptr(), direct_snapshot_buffer_.data(),
            tensor_bytes_, cudaMemcpyHostToDevice, *cuda_stream_);
          if (result != cudaSuccess) throw std::runtime_error(cudaGetErrorString(result));
          result = cudaStreamSynchronize(*cuda_stream_);
          if (result != cudaSuccess) throw std::runtime_error(cudaGetErrorString(result));
        }
      } else {
        cuda_backend_->snapshot(
          reinterpret_cast<float *>(write_handle.get_ptr()), window_end_us, *cuda_stream_);
      }
    }
    // Timestamp the tensor with the freshest event actually represented in it.
    // The logical fixed-rate window end can be ahead of the newest received
    // event, and the packet-derived sensor/ROS offset can move slightly between
    // packets.  Using that future window end as the header would create negative
    // sensor-to-command latency that later gets clamped to zero.  Capping the
    // source stamp at the scheduled publication time preserves causality while
    // making the downstream metric an age-of-information measurement.
    const auto scheduled_ros_ns = started_ros_ns >= static_cast<std::int64_t>(lateness_ns) ?
      started_ros_ns - static_cast<std::int64_t>(lateness_ns) : started_ros_ns;
    const auto latest_event_ros_ns =
      last_applied_event_ros_ns_.load(std::memory_order_relaxed);
    auto timestamp_ns = latest_event_ros_ns > 0 ?
      std::min(scheduled_ros_ns, latest_event_ros_ns) : scheduled_ros_ns;
    timestamp_ns = std::max(timestamp_ns, last_header_timestamp_ns_ + std::int64_t{1});
    std_msgs::msg::Header header;
    header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000LL);
    header.stamp.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1000000000LL);
    header.frame_id = frame_id_;
    const auto snapshot_ros_ns = now().nanoseconds();
    const auto window_end_age_ns = snapshot_ros_ns >= scheduled_ros_ns ?
      static_cast<std::uint64_t>(snapshot_ros_ns - scheduled_ros_ns) : 0U;
    window_end_age_ns_.store(window_end_age_ns, std::memory_order_relaxed);
    update_max(window_end_age_max_ns_, window_end_age_ns);
    auto message = make_tensor_list(header, tensor_name_, std::move(tensor));
    tensor_publisher_->publish(message);
    tensor_timing_->publish(header, window_end_us-window_us_, window_end_us,
      sensor_to_ros_offset_ns_, (latest_event_ros_ns-sensor_to_ros_offset_ns_)/1000LL,
      tensor_name_, channels_, height_, width_);
    last_header_timestamp_ns_ = timestamp_ns;
    published_tensors_.fetch_add(1, std::memory_order_relaxed);

    if (snapshot_event_version_ == applied_event_version_) {
      reused_snapshots_.fetch_add(1, std::memory_order_relaxed);
    }
    snapshot_event_version_ = applied_event_version_;
    events_per_snapshot_.store(events_since_snapshot_, std::memory_order_relaxed);
    update_max(events_per_snapshot_max_, events_since_snapshot_);
    events_since_snapshot_ = 0U;

    if (last_publish_at_ != SteadyTime{}) {
      const auto interval_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(started - last_publish_at_).count());
      publish_interval_ns_.fetch_add(interval_ns, std::memory_order_relaxed);
      publish_interval_samples_.fetch_add(1, std::memory_order_relaxed);
      update_max(publish_interval_max_ns_, interval_ns);
    }
    last_publish_at_ = started;
    const auto event_ros_ns = last_applied_event_ros_ns_.load(std::memory_order_relaxed);
    const auto current_ros_ns = now().nanoseconds();
    if (event_ros_ns > 0 && current_ros_ns >= event_ros_ns) {
      const auto age_ns = static_cast<std::uint64_t>(current_ros_ns - event_ros_ns);
      state_age_ns_.store(age_ns, std::memory_order_relaxed);
      update_max(state_age_max_ns_, age_ns);
    }
  } catch (const std::exception & error) {
    if (event_aggregation_ == "evt3_runs_gpu") {
      std::lock_guard<std::mutex> lock(direct_mutex_);
      direct_histogram_->mark_all_dirty();
    }
    const std::string message = error.what();
    if (message.find("exhaust") != std::string::npos ||
      message.find("pool") != std::string::npos)
    {
      tensor_allocation_errors_.fetch_add(1, std::memory_order_relaxed);
    }
    publish_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Asynchronous event tensor snapshot failed: %s", error.what());
  }
  const auto elapsed_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - started).count());
  snapshot_time_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed);
  update_max(snapshot_time_max_ns_, elapsed_ns);
  if (milliseconds(elapsed_ns) > deadline_ms_) {
    deadline_misses_.fetch_add(1, std::memory_order_relaxed);
  }
}

void AsyncEventTensorPreprocessorNode::eventCD(
  const std::uint64_t sensor_time, const std::uint16_t x, const std::uint16_t y,
  const std::uint8_t polarity)
{
  if (x >= output_x_lut_.size() || y >= output_y_lut_.size()) {
    out_of_bounds_events_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const auto timestamp_us = static_cast<std::int64_t>(sensor_time / 1000ULL);
  if (decode_last_event_us_ != 0 && timestamp_us < decode_last_event_us_) {
    const auto backward_us = static_cast<std::uint64_t>(decode_last_event_us_ - timestamp_us);
    out_of_order_events_.fetch_add(1, std::memory_order_relaxed);
    update_max(backward_jump_max_us_, backward_us);
    if (backward_us <= static_cast<std::uint64_t>(timestamp_backward_tolerance_us_)) {
      dropped_out_of_order_events_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    timestamp_resets_.fetch_add(1, std::memory_order_relaxed);
    request_decode_reset();
  }
  decode_last_event_us_ = timestamp_us;
  decode_scratch_->push_back(CudaEvent{
      timestamp_us, output_x_lut_[x], output_y_lut_[y],
      static_cast<std::uint8_t>(polarity != 0),
      {0, 0, 0}});
}

void AsyncEventTensorPreprocessorNode::rebuild_coordinate_luts(
  const std::uint32_t packet_width, const std::uint32_t packet_height)
{
  packet_width_ = packet_width;
  packet_height_ = packet_height;
  output_x_lut_.resize(packet_width_);
  output_y_lut_.resize(packet_height_);

  for (std::uint32_t x = 0; x < packet_width_; ++x) {
    const auto scaled = static_cast<std::uint64_t>(x) * static_cast<std::uint64_t>(width_) /
      packet_width_;
    output_x_lut_[x] = static_cast<std::uint16_t>(std::min<std::uint64_t>(
          static_cast<std::uint64_t>(width_ - 1), scaled));
  }
  for (std::uint32_t y = 0; y < packet_height_; ++y) {
    const auto scaled = static_cast<std::uint64_t>(y) * static_cast<std::uint64_t>(height_) /
      packet_height_;
    output_y_lut_[y] = static_cast<std::uint16_t>(std::min<std::uint64_t>(
          static_cast<std::uint64_t>(height_ - 1), scaled));
  }
  coordinate_lut_rebuilds_.fetch_add(1, std::memory_order_relaxed);
}

void AsyncEventTensorPreprocessorNode::request_decode_reset()
{
  decode_reset_pending_ = true;
  decode_last_event_us_ = 0;
  if (decode_scratch_) {
    decode_scratch_->clear();
  }
}

bool AsyncEventTensorPreprocessorNode::eventExtTrigger(
  const std::uint64_t, const std::uint8_t, const std::uint8_t)
{
  return true;
}

void AsyncEventTensorPreprocessorNode::finished()
{
}

void AsyncEventTensorPreprocessorNode::rawData(const char *, const std::size_t)
{
}

std::int64_t AsyncEventTensorPreprocessorNode::ros_timestamp_ns(
  const std::int64_t sensor_timestamp_us) const
{
  return sensor_timestamp_us * 1000LL + sensor_to_ros_offset_ns_;
}

std::int64_t AsyncEventTensorPreprocessorNode::aligned_window_end(
  const std::int64_t target_us) const
{
  if (publish_schedule_origin_us_ == 0 || target_us <= publish_schedule_origin_us_) {
    return publish_schedule_origin_us_;
  }
  const auto target_offset = target_us - publish_schedule_origin_us_;
  const auto stride_steps = (target_offset + stride_us_ / 2) / stride_us_;
  return publish_schedule_origin_us_ + stride_steps * stride_us_;
}

void AsyncEventTensorPreprocessorNode::publish_diagnostics()
{
  const auto now_steady = std::chrono::steady_clock::now();
  const auto elapsed_s = std::chrono::duration<double>(
    now_steady - last_statistics_time_).count();
  last_statistics_time_ = now_steady;
  if (elapsed_s <= 0.0) {
    return;
  }

  const auto packets = received_packets_.exchange(0, std::memory_order_relaxed);
  const auto packet_interarrival_samples =
    packet_interarrival_samples_.exchange(0, std::memory_order_relaxed);
  const auto packet_interarrival_ns =
    packet_interarrival_ns_.exchange(0, std::memory_order_relaxed);
  const auto packet_interarrival_max_ns =
    packet_interarrival_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto packet_interarrival_over_output_period =
    packet_interarrival_over_output_period_.exchange(0, std::memory_order_relaxed);
  const auto enqueued = enqueued_packets_.exchange(0, std::memory_order_relaxed);
  const auto packet_drops =
    dropped_packet_queue_packets_.exchange(0, std::memory_order_relaxed);
  const auto packet_drop_bytes =
    dropped_packet_queue_bytes_.exchange(0, std::memory_order_relaxed);
  const auto stale_packets =
    stale_packet_queue_packets_.exchange(0, std::memory_order_relaxed);
  const auto callback_ns = packet_callback_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto callback_max_ns =
    packet_callback_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto decoded_packets = decoded_packets_.exchange(0, std::memory_order_relaxed);
  const auto events = decoded_events_.exchange(0, std::memory_order_relaxed);
  const auto decode_errors = decode_errors_.exchange(0, std::memory_order_relaxed);
  const auto decode_ns = decode_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_max_ns = decode_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_events_at_time_max =
    decode_events_at_time_max_.exchange(0, std::memory_order_relaxed);
  const auto decode_packet_events_max =
    decode_packet_events_max_.exchange(0, std::memory_order_relaxed);
  const auto decode_time_at_events_max_ns =
    decode_time_at_events_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_correlation_samples =
    decode_correlation_samples_.exchange(0, std::memory_order_relaxed);
  const auto decode_correlation_events_sum =
    decode_correlation_events_sum_.exchange(0, std::memory_order_relaxed);
  const auto decode_correlation_time_ns_sum =
    decode_correlation_time_ns_sum_.exchange(0, std::memory_order_relaxed);
  const auto decode_correlation_events_squared_sum =
    decode_correlation_events_squared_sum_.exchange(0, std::memory_order_relaxed);
  const auto decode_correlation_time_ns_squared_sum =
    decode_correlation_time_ns_squared_sum_.exchange(0, std::memory_order_relaxed);
  const auto decode_correlation_cross_sum =
    decode_correlation_cross_sum_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_growth_packets =
    decode_buffer_growth_packets_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_growth_events =
    decode_buffer_growth_events_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_growth_events_max =
    decode_buffer_growth_events_max_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_capacity_before_events =
    decode_buffer_capacity_before_events_.load(std::memory_order_relaxed);
  const auto decode_buffer_capacity_after_events_max =
    decode_buffer_capacity_after_events_max_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_pool_acquires =
    decode_buffer_pool_acquires_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_pool_hits =
    decode_buffer_pool_hits_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_pool_misses =
    decode_buffer_pool_misses_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_pool_returns =
    decode_buffer_pool_returns_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_pool_discarded_full =
    decode_buffer_pool_discarded_full_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_pool_discarded_oversize =
    decode_buffer_pool_discarded_oversize_.exchange(0, std::memory_order_relaxed);
  const auto decode_buffer_pool_depth_max =
    decode_buffer_pool_depth_max_.exchange(0, std::memory_order_relaxed);
  const auto decode_service_calls = decode_service_calls_.exchange(0, std::memory_order_relaxed);
  const auto decode_service_ns = decode_service_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_service_max_ns =
    decode_service_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_handoff_ns = decode_handoff_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_handoff_max_ns =
    decode_handoff_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_thread_cpu_ns =
    decode_thread_cpu_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_thread_cpu_max_ns =
    decode_thread_cpu_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_scheduling_ns =
    decode_scheduling_delay_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_scheduling_max_ns =
    decode_scheduling_delay_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_cpu_migrations = decode_cpu_migrations_.exchange(0, std::memory_order_relaxed);
  const auto decode_wait_ns = decode_queue_wait_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_wait_max_ns = decode_queue_wait_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto decoded_drops = dropped_decoded_batches_.exchange(0, std::memory_order_relaxed);
  const auto event_drops = dropped_decoded_events_.exchange(0, std::memory_order_relaxed);
  const auto stale_decoded = stale_decoded_batches_.exchange(0, std::memory_order_relaxed);
  const auto stale_events = stale_decoded_events_.exchange(0, std::memory_order_relaxed);
  const auto gpu_updates = gpu_updates_.exchange(0, std::memory_order_relaxed);
  const auto gpu_events = gpu_events_.exchange(0, std::memory_order_relaxed);
  const auto gpu_ns = gpu_update_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto gpu_max_ns = gpu_update_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto gpu_wait_ns = gpu_queue_wait_ns_.exchange(0, std::memory_order_relaxed);
  const auto gpu_wait_max_ns = gpu_queue_wait_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto tensors = published_tensors_.exchange(0, std::memory_order_relaxed);
  const auto publish_errors = publish_errors_.exchange(0, std::memory_order_relaxed);
  const auto pool_exhaustions = tensor_allocation_errors_.exchange(0, std::memory_order_relaxed);
  const auto snapshot_ns = snapshot_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto snapshot_max_ns = snapshot_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto lateness_ns = snapshot_lateness_ns_.exchange(0, std::memory_order_relaxed);
  const auto lateness_max_ns = snapshot_lateness_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto interval_ns = publish_interval_ns_.exchange(0, std::memory_order_relaxed);
  const auto interval_max_ns = publish_interval_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto interval_samples = publish_interval_samples_.exchange(0, std::memory_order_relaxed);
  const auto deadline_misses = deadline_misses_.exchange(0, std::memory_order_relaxed);
  const auto skipped = skipped_windows_.exchange(0, std::memory_order_relaxed);
  const auto reused = reused_snapshots_.exchange(0, std::memory_order_relaxed);
  const auto events_snapshot = events_per_snapshot_.load(std::memory_order_relaxed);
  const auto events_snapshot_max = events_per_snapshot_max_.exchange(0, std::memory_order_relaxed);
  const auto out_of_bounds = out_of_bounds_events_.exchange(0, std::memory_order_relaxed);
  const auto out_of_order = out_of_order_events_.exchange(0, std::memory_order_relaxed);
  const auto dropped_ooo = dropped_out_of_order_events_.exchange(0, std::memory_order_relaxed);
  const auto resets = timestamp_resets_.exchange(0, std::memory_order_relaxed);
  const auto backward_max = backward_jump_max_us_.exchange(0, std::memory_order_relaxed);
  const auto packet_depth_max = packet_queue_depth_max_.exchange(0, std::memory_order_relaxed);
  const auto input_sequence_gap_occurrences =
    input_sequence_gap_occurrences_.exchange(0, std::memory_order_relaxed);
  const auto input_sequence_missing_packets =
    input_sequence_missing_packets_.exchange(0, std::memory_order_relaxed);
  const auto input_sequence_gap_max =
    input_sequence_gap_max_.exchange(0, std::memory_order_relaxed);
  const auto input_sequence_reorders =
    input_sequence_reorders_.exchange(0, std::memory_order_relaxed);
  const auto decoded_depth_max = decoded_queue_depth_max_.exchange(0, std::memory_order_relaxed);
  const auto state_age_max = state_age_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto window_end_age_max =
    window_end_age_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto packet_arrival_ns = last_packet_arrival_steady_ns_.load(std::memory_order_relaxed);
  const auto packet_age_ms = packet_arrival_ns > 0 ?
    milliseconds(static_cast<std::uint64_t>(steady_nanoseconds(now_steady) - packet_arrival_ns)) :
    -1.0;
  const auto decode_cpu = decode_last_cpu_.load(std::memory_order_relaxed);
  const auto decode_cpu_cur_khz = read_cpu_frequency_khz(decode_cpu, "scaling_cur_freq");
  const auto decode_cpu_min_khz = read_cpu_frequency_khz(decode_cpu, "scaling_min_freq");
  const auto decode_cpu_max_khz = read_cpu_frequency_khz(decode_cpu, "scaling_max_freq");
  double decode_event_count_time_correlation = 0.0;
  if (decode_correlation_samples >= 2U) {
    const auto count = static_cast<long double>(decode_correlation_samples);
    const auto sum_events = static_cast<long double>(decode_correlation_events_sum);
    const auto sum_time = static_cast<long double>(decode_correlation_time_ns_sum);
    const auto event_variance =
      count * static_cast<long double>(decode_correlation_events_squared_sum) -
      sum_events * sum_events;
    const auto time_variance =
      count * static_cast<long double>(decode_correlation_time_ns_squared_sum) -
      sum_time * sum_time;
    if (event_variance > 0.0L && time_variance > 0.0L) {
      const auto covariance =
        count * static_cast<long double>(decode_correlation_cross_sum) -
        sum_events * sum_time;
      decode_event_count_time_correlation = static_cast<double>(
        covariance / std::sqrt(event_variance * time_variance));
      decode_event_count_time_correlation = std::clamp(
        decode_event_count_time_correlation, -1.0, 1.0);
    }
  }

  const bool healthy =
    packet_drops == 0 && stale_packets == 0 && decoded_drops == 0 &&
    stale_decoded == 0 && decode_errors == 0 &&
    publish_errors == 0 && pool_exhaustions == 0 && deadline_misses == 0 && resets == 0 &&
    input_sequence_missing_packets == 0 && input_sequence_reorders == 0;
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string("/async_event_tensor_preprocessor");
  status.hardware_id = "gpu";
  status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = healthy ? "fixed-rate pipeline healthy" :
    "event queue, timestamp, CUDA pool, or fixed-rate deadline issue detected";
  status.values = {
    value("pipeline_mode", "single_node_async"),
    value("event_aggregation", event_aggregation_),
    value("tensor_upload", event_aggregation_ == "evt3_runs_gpu" ? "dirty_cpu_bins" :
      (direct_histogram_ ? "full_cpu_snapshot" : "gpu_ring_snapshot")),
    number("packets_received_per_s", packets / elapsed_s),
    number(
      "packets_received_total",
      received_packets_total_.load(std::memory_order_relaxed)),
    number(
      "last_received_sequence",
      last_received_sequence_.load(std::memory_order_relaxed)),
    number("input_sequence_gap_occurrences", input_sequence_gap_occurrences),
    number("input_sequence_missing_packets", input_sequence_missing_packets),
    number("input_sequence_gap_max", input_sequence_gap_max),
    number("input_sequence_reorders", input_sequence_reorders),
    number("packets_enqueued_per_s", enqueued / elapsed_s),
    number(
      "packet_interarrival_ms_avg", packet_interarrival_samples == 0 ? 0.0 :
      milliseconds(packet_interarrival_ns) / packet_interarrival_samples),
    number("packet_interarrival_ms_max", milliseconds(packet_interarrival_max_ns)),
    number(
      "packet_interarrival_over_output_period",
      packet_interarrival_over_output_period),
    number("events_decoded_per_s", events / elapsed_s),
    number("tensors_per_s", tensors / elapsed_s),
    number("packet_callback_ms_avg", packets == 0 ? 0.0 : milliseconds(callback_ns) / packets),
    number("packet_callback_ms_max", milliseconds(callback_max_ns)),
    number("packet_callback_busy_pct", 100.0 * callback_ns / (elapsed_s * 1.0e9)),
    number("packet_queue_depth", packet_queue_depth_.load(std::memory_order_relaxed)),
    number("packet_queue_depth_max", packet_depth_max),
    number("packet_queue_capacity", packet_queue_capacity_),
    number("packet_queue_dropped_packets", packet_drops),
    number("packet_queue_stale_packets", stale_packets),
    number("packet_queue_dropped_bytes", packet_drop_bytes),
    number(
      "packet_queue_wait_ms_avg", decoded_packets + stale_packets == 0 ? 0.0 :
      milliseconds(decode_wait_ns) / (decoded_packets + stale_packets)),
    number("packet_queue_wait_ms_max", milliseconds(decode_wait_max_ns)),
    number("decode_ms_avg", decoded_packets == 0 ? 0.0 : milliseconds(decode_ns) / decoded_packets),
    number("decode_ms_max", milliseconds(decode_max_ns)),
    number("decode_ns_per_event", events == 0 ? 0.0 : static_cast<double>(decode_ns) / events),
    number(
      "decode_packet_events_avg", decoded_packets == 0 ? 0.0 :
      static_cast<double>(events) / decoded_packets),
    number("decode_packet_events_max", decode_packet_events_max),
    number("decode_events_at_decode_ms_max", decode_events_at_time_max),
    number("decode_ms_for_largest_event_packet", milliseconds(decode_time_at_events_max_ns)),
    number("decode_event_count_time_correlation", decode_event_count_time_correlation),
    number("decode_correlation_samples", decode_correlation_samples),
    number("decode_buffer_growth_packets", decode_buffer_growth_packets),
    number(
      "decode_buffer_growth_pct", decoded_packets == 0 ? 0.0 :
      100.0 * static_cast<double>(decode_buffer_growth_packets) / decoded_packets),
    number(
      "decode_buffer_growth_events_avg", decode_buffer_growth_packets == 0 ? 0.0 :
      static_cast<double>(decode_buffer_growth_events) / decode_buffer_growth_packets),
    number("decode_buffer_growth_events_max", decode_buffer_growth_events_max),
    number("decode_buffer_capacity_before_events", decode_buffer_capacity_before_events),
    number("decode_buffer_capacity_after_events_max", decode_buffer_capacity_after_events_max),
    number("decode_buffer_pool_acquires", decode_buffer_pool_acquires),
    number("decode_buffer_pool_hits", decode_buffer_pool_hits),
    number("decode_buffer_pool_misses", decode_buffer_pool_misses),
    number(
      "decode_buffer_pool_hit_pct", decode_buffer_pool_acquires == 0 ? 0.0 :
      100.0 * static_cast<double>(decode_buffer_pool_hits) / decode_buffer_pool_acquires),
    number("decode_buffer_pool_returns", decode_buffer_pool_returns),
    number("decode_buffer_pool_discarded_full", decode_buffer_pool_discarded_full),
    number("decode_buffer_pool_discarded_oversize", decode_buffer_pool_discarded_oversize),
    number("decode_buffer_pool_depth", decode_buffer_pool_depth_.load(std::memory_order_relaxed)),
    number("decode_buffer_pool_depth_max", decode_buffer_pool_depth_max),
    number("decode_buffer_pool_capacity", decode_buffer_pool_capacity_),
    number(
      "decode_buffer_pool_retained_events",
      decode_buffer_pool_retained_events_.load(std::memory_order_relaxed)),
    number(
      "decode_buffer_pool_retained_bytes",
      decode_buffer_pool_retained_events_.load(std::memory_order_relaxed) * sizeof(CudaEvent)),
    number("decode_buffer_pool_max_events", decode_buffer_pool_max_events_),
    number(
      "decode_buffer_pool_max_retained_bytes",
      decode_buffer_pool_capacity_ * decode_buffer_pool_max_events_ * sizeof(CudaEvent)),
    number(
      "decode_handoff_ms_avg", decoded_packets == 0 ? 0.0 :
      milliseconds(decode_handoff_ns) / decoded_packets),
    number("decode_handoff_ms_max", milliseconds(decode_handoff_max_ns)),
    number("decode_service_calls", decode_service_calls),
    number(
      "decode_service_ms_avg", decode_service_calls == 0 ? 0.0 :
      milliseconds(decode_service_ns) / decode_service_calls),
    number("decode_service_ms_max", milliseconds(decode_service_max_ns)),
    number(
      "decode_service_ns_per_event", events == 0 ? 0.0 :
      static_cast<double>(decode_service_ns) / events),
    number(
      "decode_thread_cpu_ms_avg", decode_service_calls == 0 ? 0.0 :
      milliseconds(decode_thread_cpu_ns) / decode_service_calls),
    number("decode_thread_cpu_ms_max", milliseconds(decode_thread_cpu_max_ns)),
    number(
      "decode_scheduling_delay_ms_avg", decode_service_calls == 0 ? 0.0 :
      milliseconds(decode_scheduling_ns) / decode_service_calls),
    number("decode_scheduling_delay_ms_max", milliseconds(decode_scheduling_max_ns)),
    number(
      "decode_scheduling_delay_pct", decode_service_ns == 0 ? 0.0 :
      100.0 * static_cast<double>(decode_scheduling_ns) / decode_service_ns),
    number("decoder_callback_busy_pct", 100.0 * decode_ns / (elapsed_s * 1.0e9)),
    number("decode_worker_busy_pct", 100.0 * decode_service_ns / (elapsed_s * 1.0e9)),
    number("decode_thread_cpu_busy_pct", 100.0 * decode_thread_cpu_ns / (elapsed_s * 1.0e9)),
    value("decode_busy_definition", "full_service_wall_time"),
    number("decode_cpu_migrations", decode_cpu_migrations),
    number("decode_last_cpu", decode_cpu),
    number("decode_cpu_scaling_cur_khz", decode_cpu_cur_khz),
    number("decode_cpu_scaling_min_khz", decode_cpu_min_khz),
    number("decode_cpu_scaling_max_khz", decode_cpu_max_khz),
    value("decode_cpu_frequency_sampling", "diagnostics_interval"),
    number("decode_errors", decode_errors),
    number("decoded_queue_depth", decoded_queue_depth_.load(std::memory_order_relaxed)),
    number("decoded_queue_depth_max", decoded_depth_max),
    number("decoded_queue_capacity", decoded_queue_capacity_),
    number("decoded_queue_dropped_batches", decoded_drops),
    number("decoded_queue_dropped_events", event_drops),
    number("decoded_queue_stale_batches", stale_decoded),
    number("decoded_queue_stale_events", stale_events),
    number("max_queue_age_ms", max_queue_age_ms_),
    number("gpu_updates", gpu_updates),
    number("gpu_events", gpu_events),
    value(
      "gpu_timing_scope",
      "host_submit_including_staging_slot_backpressure; not pure CUDA kernel elapsed time"),
    number("gpu_update_ms_avg", gpu_updates == 0 ? 0.0 : milliseconds(gpu_ns) / gpu_updates),
    number("gpu_update_ms_max", milliseconds(gpu_max_ns)),
    number("gpu_worker_busy_pct", 100.0 * (gpu_ns + snapshot_ns) / (elapsed_s * 1.0e9)),
    number("gpu_queue_wait_ms_avg", gpu_updates == 0 ? 0.0 : milliseconds(gpu_wait_ns) / gpu_updates),
    number("gpu_queue_wait_ms_max", milliseconds(gpu_wait_max_ns)),
    number("snapshot_ms_avg", tensors == 0 ? 0.0 : milliseconds(snapshot_ns) / tensors),
    number("snapshot_ms_max", milliseconds(snapshot_max_ns)),
    number("snapshot_lateness_ms_avg", tensors == 0 ? 0.0 : milliseconds(lateness_ns) / tensors),
    number("snapshot_lateness_ms_max", milliseconds(lateness_max_ns)),
    number("publish_interval_ms_avg", interval_samples == 0 ? 0.0 : milliseconds(interval_ns) / interval_samples),
    number("publish_interval_ms_max", milliseconds(interval_max_ns)),
    number("deadline_ms", deadline_ms_),
    number("deadline_misses", deadline_misses),
    number("fixed_rate_skipped_windows", skipped),
    number("reused_event_state_snapshots", reused),
    number("fresh_event_state_snapshots", tensors >= reused ? tensors - reused : 0U),
    number(
      "event_update_absent_pct", tensors == 0 ? 0.0 :
      100.0 * static_cast<double>(reused) / tensors),
    number("events_per_snapshot", events_snapshot),
    number("events_per_snapshot_max", events_snapshot_max),
    number("event_state_age_ms", milliseconds(state_age_ns_.load(std::memory_order_relaxed))),
    number("event_state_age_ms_max", milliseconds(state_age_max)),
    number(
      "window_end_age_ms", milliseconds(window_end_age_ns_.load(std::memory_order_relaxed))),
    number("window_end_age_ms_max", milliseconds(window_end_age_max)),
    number(
      "schedule_initial_lag_ms",
      milliseconds(schedule_initial_lag_ns_.load(std::memory_order_relaxed))),
    number("tensor_allocation_errors", pool_exhaustions),
    number("publish_errors", publish_errors),
    number("out_of_bounds_events", out_of_bounds),
    number("out_of_order_events", out_of_order),
    number("dropped_out_of_order_events", dropped_ooo),
    number("timestamp_resets", resets),
    number("max_backward_jump_us", backward_max),
    number("timestamp_backward_tolerance_us", timestamp_backward_tolerance_us_),
    number("last_packet_arrival_age_ms", packet_age_ms),
    value("coordinate_mapping", "lookup_table"),
    number("coordinate_lut_rebuilds", coordinate_lut_rebuilds_.load(std::memory_order_relaxed)),
    number("bins", bins_),
    number("channels", channels_),
    number("window_ms", window_us_ / 1000.0),
    number("representation_stride_ms", stride_us_ / 1000.0),
    number("target_output_hz", 1.0e6 / output_period_us_),
    value("snapshot_schedule", "nearest_representation_stride"),
    value("tensor_header_timestamp_source", "freshest_event_capped_at_schedule"),
    number("gpu_chunk_events", gpu_chunk_events_),
    number("tensor_bytes", tensor_bytes_),
    value("polarity_mode", polarity_mode_),
    value("polarity_layout", polarity_layout_),
    value("publish_empty", publish_empty_ ? "true" : "false")};
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);

  debug_ = get_parameter("debug").as_bool();
  if (debug_) {
    RCLCPP_INFO(
      get_logger(),
      "async event preprocess: %.1f packet/s %.1f Mevent/s %.1f tensor/s; "
      "queues packet=%lu decoded=%lu; callback %.3fms decode %.3f/%.3fms "
      "gpu %.3f/%.3fms snapshot %.3f/%.3fms interval %.3f/%.3fms; "
      "skipped=%lu reused=%lu drops=%lu/%lu seq_missing=%lu "
      "seq_reorders=%lu state_age=%.3fms",
      packets / elapsed_s, events / elapsed_s / 1.0e6, tensors / elapsed_s,
      static_cast<unsigned long>(packet_queue_depth_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(decoded_queue_depth_.load(std::memory_order_relaxed)),
      packets == 0 ? 0.0 : milliseconds(callback_ns) / packets,
      decoded_packets == 0 ? 0.0 : milliseconds(decode_ns) / decoded_packets,
      milliseconds(decode_max_ns),
      gpu_updates == 0 ? 0.0 : milliseconds(gpu_ns) / gpu_updates,
      milliseconds(gpu_max_ns), tensors == 0 ? 0.0 : milliseconds(snapshot_ns) / tensors,
      milliseconds(snapshot_max_ns),
      interval_samples == 0 ? 0.0 : milliseconds(interval_ns) / interval_samples,
      milliseconds(interval_max_ns), static_cast<unsigned long>(skipped),
      static_cast<unsigned long>(reused), static_cast<unsigned long>(packet_drops),
      static_cast<unsigned long>(event_drops),
      static_cast<unsigned long>(input_sequence_missing_packets),
      static_cast<unsigned long>(input_sequence_reorders),
      milliseconds(state_age_ns_.load(std::memory_order_relaxed)));
  }
}

}  // namespace openeb_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(
  openeb_ros2::AsyncEventTensorPreprocessorNode)
