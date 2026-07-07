#include "openeb_ros2/preprocessor_component.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace openeb_ros2
{
namespace
{

void update_max(
  std::atomic<std::uint64_t> & maximum, const std::uint64_t value)
{
  auto current = maximum.load(std::memory_order_relaxed);
  while (
    current < value &&
    !maximum.compare_exchange_weak(
      current, value, std::memory_order_relaxed))
  {
  }
}

}  // namespace

PreprocessorComponent::PreprocessorComponent(const rclcpp::NodeOptions & options)
: Node("openeb_preprocessor", options)
{
  expected_encoding_ =
    declare_parameter<std::string>("expected_encoding", "evt3");
  output_frame_id_ =
    declare_parameter<std::string>("output_frame_id", "");
  drop_empty_packets_ =
    declare_parameter<bool>("drop_empty_packets", true);
  drop_unexpected_encoding_ =
    declare_parameter<bool>("drop_unexpected_encoding", true);
  event_image_enabled_ =
    declare_parameter<bool>("event_image_enabled", true);
  event_image_fps_ =
    declare_parameter<double>("event_image_fps", 25.0);
  event_image_encoding_ =
    declare_parameter<std::string>("event_image_encoding", "bgr8");
  event_image_publish_empty_ =
    declare_parameter<bool>("event_image_publish_empty", true);

  const auto subscription_depth =
    declare_parameter<std::int64_t>("subscription_depth", 8);
  const auto publisher_depth =
    declare_parameter<std::int64_t>("publisher_depth", 8);
  const auto event_image_publisher_depth =
    declare_parameter<std::int64_t>("event_image_publisher_depth", 2);
  statistics_interval_s_ =
    declare_parameter<double>("statistics_interval_s", 1.0);
  if (
    subscription_depth <= 0 || publisher_depth <= 0 ||
    event_image_publisher_depth <= 0)
  {
    throw std::invalid_argument("QoS queue depths must be positive");
  }
  if (
    event_image_enabled_ &&
    (!std::isfinite(event_image_fps_) || event_image_fps_ <= 0.0))
  {
    throw std::invalid_argument(
      "event_image_fps must be finite and positive when event image output is enabled");
  }
  if (
    event_image_encoding_ != "bgr8" &&
    event_image_encoding_ != "mono8")
  {
    throw std::invalid_argument(
      "event_image_encoding must be either 'bgr8' or 'mono8'");
  }
  if (statistics_interval_s_ < 0.0) {
    throw std::invalid_argument("statistics_interval_s must be non-negative");
  }

  const auto subscription_qos = rclcpp::QoS(rclcpp::KeepLast(
      static_cast<std::size_t>(subscription_depth)))
                                  .best_effort()
                                  .durability_volatile();
  const auto publisher_qos = rclcpp::QoS(rclcpp::KeepLast(
      static_cast<std::size_t>(publisher_depth)))
                               .best_effort()
                               .durability_volatile();
  const auto image_publisher_qos = rclcpp::QoS(rclcpp::KeepLast(
      static_cast<std::size_t>(event_image_publisher_depth)))
                                     .reliable()
                                     .durability_volatile();

  event_publisher_ =
    create_publisher<EventPacket>("events", publisher_qos);
  event_subscription_ = create_subscription<EventPacket>(
    "events_raw", subscription_qos,
    std::bind(&PreprocessorComponent::on_packet, this, std::placeholders::_1));

  if (event_image_enabled_) {
    event_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(
      "event_image", image_publisher_qos);
    event_decoder_factory_ = std::make_unique<EventDecoderFactory>();
    event_image_channels_ = event_image_encoding_ == "bgr8" ? 3U : 1U;
    event_image_background_value_ =
      event_image_encoding_ == "bgr8" ? 0U : 127U;

    const auto image_period_ns = std::chrono::nanoseconds(
      std::max<std::int64_t>(
        1, static_cast<std::int64_t>(1.0e9 / event_image_fps_)));
    event_image_timer_ = create_wall_timer(
      image_period_ns,
      std::bind(&PreprocessorComponent::on_event_image_timer, this));
  }

  last_statistics_time_ = std::chrono::steady_clock::now();
  if (statistics_interval_s_ > 0.0) {
    const auto period_ms = std::chrono::milliseconds(
      std::max<std::int64_t>(
        1, static_cast<std::int64_t>(statistics_interval_s_ * 1000.0)));
    statistics_timer_ = create_wall_timer(
      period_ms, std::bind(&PreprocessorComponent::print_statistics, this));
  }
}

void PreprocessorComponent::on_packet(EventPacket::UniquePtr packet)
{
  if (!packet) {
    return;
  }

  const auto callback_start = std::chrono::steady_clock::now();
  const auto packet_bytes = packet->events.size();
  received_messages_.fetch_add(1, std::memory_order_relaxed);
  received_bytes_.fetch_add(packet_bytes, std::memory_order_relaxed);

  const auto receive_time = now();
  const rclcpp::Time source_time(
    packet->header.stamp, get_clock()->get_clock_type());
  const auto latency_ns = (receive_time - source_time).nanoseconds();
  if (latency_ns >= 0) {
    const auto latency = static_cast<std::uint64_t>(latency_ns);
    transport_latency_ns_.fetch_add(latency, std::memory_order_relaxed);
    transport_latency_samples_.fetch_add(1, std::memory_order_relaxed);
    update_max(transport_latency_max_ns_, latency);
  }

  if (!preprocess(*packet)) {
    record_callback_time(callback_start);
    return;
  }

  if (event_image_enabled_) {
    const bool geometry_changed =
      event_image_width_ != packet->width ||
      event_image_height_ != packet->height;
    if (geometry_changed) {
      const bool was_initialized =
        event_image_width_ != 0 || event_image_height_ != 0;
      event_image_width_ = packet->width;
      event_image_height_ = packet->height;
      reset_event_image_decoder();
      if (event_image_subscriber_active_) {
        start_event_image();
      }
      if (was_initialized) {
        RCLCPP_WARN(
          get_logger(), "Event image geometry changed to %ux%u; decoder reset",
          event_image_width_, event_image_height_);
      }
    }
    event_image_frame_id_ = packet->header.frame_id;
    if (event_image_subscriber_active_) {
      decode_for_event_image(*packet);
    } else {
      image_no_subscriber_packets_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  if (!has_output_subscribers()) {
    no_subscriber_messages_.fetch_add(1, std::memory_order_relaxed);
    record_callback_time(callback_start);
    return;
  }

  published_messages_.fetch_add(1, std::memory_order_relaxed);
  published_bytes_.fetch_add(packet_bytes, std::memory_order_relaxed);
  event_publisher_->publish(std::move(packet));
  record_callback_time(callback_start);
}

bool PreprocessorComponent::preprocess(EventPacket & packet)
{
  if (drop_empty_packets_ && packet.events.empty()) {
    dropped_empty_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  if (
    drop_unexpected_encoding_ && !expected_encoding_.empty() &&
    packet.encoding != expected_encoding_)
  {
    dropped_encoding_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Dropping packet with encoding '%s'; expected '%s'",
      packet.encoding.c_str(), expected_encoding_.c_str());
    return false;
  }

  if (!output_frame_id_.empty()) {
    packet.header.frame_id = output_frame_id_;
  }

  // RAW packet pass-through remains independent from the optional decoded image.
  return true;
}

void PreprocessorComponent::eventCD(
  const std::uint64_t, const std::uint16_t x, const std::uint16_t y,
  const std::uint8_t polarity)
{
  ++decoded_events_in_packet_;
  if (!active_event_image_) {
    return;
  }
  if (x >= event_image_width_ || y >= event_image_height_) {
    ++out_of_bounds_events_in_packet_;
    return;
  }

  const auto pixel_index =
    static_cast<std::size_t>(y) * active_event_image_->step +
    static_cast<std::size_t>(x) * event_image_channels_;
  if (event_image_encoding_ == "bgr8") {
    // ON is blue, OFF is red. A pixel seeing both polarities becomes magenta.
    active_event_image_->data[pixel_index + (polarity ? 0U : 2U)] = 255U;
  } else {
    active_event_image_->data[pixel_index] = polarity ? 255U : 0U;
  }
  ++events_in_active_image_;
}

bool PreprocessorComponent::eventExtTrigger(
  const std::uint64_t, const std::uint8_t, const std::uint8_t)
{
  return true;
}

void PreprocessorComponent::finished()
{
}

void PreprocessorComponent::rawData(const char *, const std::size_t)
{
}

void PreprocessorComponent::decode_for_event_image(const EventPacket & packet)
{
  if (
    !event_decoder_factory_ || packet.width == 0 || packet.height == 0 ||
    packet.events.empty())
  {
    return;
  }

  const auto decode_start = std::chrono::steady_clock::now();
  decoded_events_in_packet_ = 0;
  out_of_bounds_events_in_packet_ = 0;

  try {
    auto * decoder = event_decoder_factory_->getInstance(packet);
    if (!decoder) {
      decode_errors_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No event image decoder for encoding '%s'", packet.encoding.c_str());
      return;
    }
    while (decoder->decode(packet, this)) {
    }
  } catch (const std::exception & error) {
    decode_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Event image decode failed: %s", error.what());
    reset_event_image_decoder();
    if (event_image_subscriber_active_) {
      start_event_image();
    }
    return;
  }

  const auto decode_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - decode_start).count());
  decoded_events_.fetch_add(
    decoded_events_in_packet_, std::memory_order_relaxed);
  out_of_bounds_events_.fetch_add(
    out_of_bounds_events_in_packet_, std::memory_order_relaxed);
  decode_calls_.fetch_add(1, std::memory_order_relaxed);
  decode_time_ns_.fetch_add(decode_ns, std::memory_order_relaxed);
  update_max(decode_time_max_ns_, decode_ns);
}

void PreprocessorComponent::on_event_image_timer()
{
  const auto timer_start = std::chrono::steady_clock::now();
  const bool has_subscribers = has_event_image_subscribers();

  if (has_subscribers != event_image_subscriber_active_) {
    event_image_subscriber_active_ = has_subscribers;
    reset_event_image_decoder();
    if (has_subscribers) {
      RCLCPP_INFO(
        get_logger(),
        "Event image output enabled for subscriber (%ux%u, encoding=%s, fps=%.1f)",
        event_image_width_, event_image_height_,
        event_image_encoding_.c_str(), event_image_fps_);
      start_event_image();
    } else {
      RCLCPP_INFO(get_logger(), "Event image output paused: no subscribers");
    }
  }

  if (!has_subscribers || event_image_width_ == 0 || event_image_height_ == 0) {
    active_event_image_.reset();
    events_in_active_image_ = 0;
  } else if (!active_event_image_) {
    start_event_image();
  } else if (event_image_publish_empty_ || events_in_active_image_ > 0) {
    active_event_image_->header.stamp = now();
    active_event_image_->header.frame_id = event_image_frame_id_;

    const auto image_bytes = active_event_image_->data.size();
    image_published_messages_.fetch_add(1, std::memory_order_relaxed);
    image_published_bytes_.fetch_add(image_bytes, std::memory_order_relaxed);
    image_published_events_.fetch_add(
      events_in_active_image_, std::memory_order_relaxed);
    event_image_publisher_->publish(std::move(active_event_image_));
    start_event_image();
  }

  const auto timer_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - timer_start).count());
  image_timer_calls_.fetch_add(1, std::memory_order_relaxed);
  image_timer_time_ns_.fetch_add(timer_ns, std::memory_order_relaxed);
  update_max(image_timer_time_max_ns_, timer_ns);
}

void PreprocessorComponent::start_event_image()
{
  if (
    !event_image_subscriber_active_ || event_image_width_ == 0 ||
    event_image_height_ == 0)
  {
    return;
  }

  const auto step =
    static_cast<std::uint64_t>(event_image_width_) * event_image_channels_;
  if (
    step > std::numeric_limits<std::uint32_t>::max() ||
    static_cast<std::uint64_t>(event_image_height_) >
    std::numeric_limits<std::size_t>::max() / step)
  {
    throw std::overflow_error("Event image dimensions are too large");
  }
  const auto image_size =
    static_cast<std::size_t>(step) * event_image_height_;

  active_event_image_ = std::make_unique<sensor_msgs::msg::Image>();
  active_event_image_->height = event_image_height_;
  active_event_image_->width = event_image_width_;
  active_event_image_->encoding = event_image_encoding_;
  active_event_image_->is_bigendian = false;
  active_event_image_->step = static_cast<std::uint32_t>(step);
  active_event_image_->data.assign(
    image_size, event_image_background_value_);
  events_in_active_image_ = 0;
}

void PreprocessorComponent::reset_event_image_decoder()
{
  active_event_image_.reset();
  event_decoder_factory_ = std::make_unique<EventDecoderFactory>();
  decoded_events_in_packet_ = 0;
  out_of_bounds_events_in_packet_ = 0;
  events_in_active_image_ = 0;
}

bool PreprocessorComponent::has_output_subscribers() const
{
  return event_publisher_->get_subscription_count() > 0 ||
         event_publisher_->get_intra_process_subscription_count() > 0;
}

bool PreprocessorComponent::has_event_image_subscribers() const
{
  return event_image_publisher_ &&
         (event_image_publisher_->get_subscription_count() > 0 ||
         event_image_publisher_->get_intra_process_subscription_count() > 0);
}

void PreprocessorComponent::record_callback_time(
  const std::chrono::steady_clock::time_point & callback_start)
{
  const auto callback_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - callback_start).count());
  callback_time_ns_.fetch_add(callback_ns, std::memory_order_relaxed);
  update_max(callback_time_max_ns_, callback_ns);
}

void PreprocessorComponent::print_statistics()
{
  const auto statistics_time = std::chrono::steady_clock::now();
  const auto elapsed_s =
    std::chrono::duration<double>(statistics_time - last_statistics_time_).count();
  last_statistics_time_ = statistics_time;
  if (elapsed_s <= 0.0) {
    return;
  }

  const auto received_messages =
    received_messages_.exchange(0, std::memory_order_relaxed);
  const auto received_bytes =
    received_bytes_.exchange(0, std::memory_order_relaxed);
  const auto published_messages =
    published_messages_.exchange(0, std::memory_order_relaxed);
  const auto published_bytes =
    published_bytes_.exchange(0, std::memory_order_relaxed);
  const auto dropped_empty =
    dropped_empty_.exchange(0, std::memory_order_relaxed);
  const auto dropped_encoding =
    dropped_encoding_.exchange(0, std::memory_order_relaxed);
  const auto no_subscriber =
    no_subscriber_messages_.exchange(0, std::memory_order_relaxed);
  const auto callback_ns =
    callback_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto callback_max_ns =
    callback_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto latency_ns =
    transport_latency_ns_.exchange(0, std::memory_order_relaxed);
  const auto latency_max_ns =
    transport_latency_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto latency_samples =
    transport_latency_samples_.exchange(0, std::memory_order_relaxed);
  const auto decoded_events =
    decoded_events_.exchange(0, std::memory_order_relaxed);
  const auto decode_calls =
    decode_calls_.exchange(0, std::memory_order_relaxed);
  const auto decode_ns =
    decode_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_max_ns =
    decode_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto decode_errors =
    decode_errors_.exchange(0, std::memory_order_relaxed);
  const auto out_of_bounds_events =
    out_of_bounds_events_.exchange(0, std::memory_order_relaxed);
  const auto image_messages =
    image_published_messages_.exchange(0, std::memory_order_relaxed);
  const auto image_bytes =
    image_published_bytes_.exchange(0, std::memory_order_relaxed);
  const auto image_events =
    image_published_events_.exchange(0, std::memory_order_relaxed);
  const auto image_timer_calls =
    image_timer_calls_.exchange(0, std::memory_order_relaxed);
  const auto image_timer_ns =
    image_timer_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto image_timer_max_ns =
    image_timer_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto image_no_subscriber_packets =
    image_no_subscriber_packets_.exchange(0, std::memory_order_relaxed);

  constexpr double bytes_per_mib = 1024.0 * 1024.0;
  constexpr double bytes_per_kib = 1024.0;
  constexpr double ns_per_us = 1000.0;

  const double receive_hz = received_messages / elapsed_s;
  const double receive_mib_s = received_bytes / bytes_per_mib / elapsed_s;
  const double average_packet_kib =
    received_messages == 0 ? 0.0 :
    static_cast<double>(received_bytes) / received_messages / bytes_per_kib;
  const double publish_hz = published_messages / elapsed_s;
  const double publish_mib_s = published_bytes / bytes_per_mib / elapsed_s;
  const double callback_mean_us =
    received_messages == 0 ? 0.0 :
    static_cast<double>(callback_ns) / received_messages / ns_per_us;
  const double callback_busy_pct =
    static_cast<double>(callback_ns) / (elapsed_s * 1.0e9) * 100.0;
  const double callback_ns_per_kib =
    received_bytes == 0 ? 0.0 :
    static_cast<double>(callback_ns) /
    (static_cast<double>(received_bytes) / bytes_per_kib);
  const double latency_mean_us =
    latency_samples == 0 ? 0.0 :
    static_cast<double>(latency_ns) / latency_samples / ns_per_us;
  const double decoded_mev_s =
    static_cast<double>(decoded_events) / 1.0e6 / elapsed_s;
  const double decode_mean_us =
    decode_calls == 0 ? 0.0 :
    static_cast<double>(decode_ns) / decode_calls / ns_per_us;
  const double decode_ns_per_event =
    decoded_events == 0 ? 0.0 :
    static_cast<double>(decode_ns) / decoded_events;
  const double image_hz = image_messages / elapsed_s;
  const double image_mib_s = image_bytes / bytes_per_mib / elapsed_s;
  const double events_per_image =
    image_messages == 0 ? 0.0 :
    static_cast<double>(image_events) / image_messages;
  const double image_timer_mean_us =
    image_timer_calls == 0 ? 0.0 :
    static_cast<double>(image_timer_ns) / image_timer_calls / ns_per_us;

  RCLCPP_INFO(
    get_logger(),
    "preprocessor_stats receive_hz=%.1f receive_mib_s=%.3f "
    "avg_packet_kib=%.2f publish_hz=%.1f publish_mib_s=%.3f "
    "callback_mean_us=%.2f callback_max_us=%.2f callback_busy_pct=%.2f "
    "callback_ns_per_kib=%.2f "
    "transport_mean_us=%.2f transport_max_us=%.2f "
    "decoded_mev_s=%.3f decode_mean_us=%.2f decode_max_us=%.2f "
    "decode_ns_per_event=%.2f image_hz=%.1f image_mib_s=%.3f "
    "events_per_image=%.1f image_timer_mean_us=%.2f "
    "image_timer_max_us=%.2f dropped_empty=%llu dropped_encoding=%llu "
    "no_subscriber=%llu image_no_subscriber=%llu decode_errors=%llu "
    "out_of_bounds_events=%llu",
    receive_hz, receive_mib_s, average_packet_kib, publish_hz,
    publish_mib_s, callback_mean_us, callback_max_ns / ns_per_us,
    callback_busy_pct, callback_ns_per_kib, latency_mean_us,
    latency_max_ns / ns_per_us, decoded_mev_s, decode_mean_us,
    decode_max_ns / ns_per_us, decode_ns_per_event, image_hz,
    image_mib_s, events_per_image, image_timer_mean_us,
    image_timer_max_ns / ns_per_us,
    static_cast<unsigned long long>(dropped_empty),
    static_cast<unsigned long long>(dropped_encoding),
    static_cast<unsigned long long>(no_subscriber),
    static_cast<unsigned long long>(image_no_subscriber_packets),
    static_cast<unsigned long long>(decode_errors),
    static_cast<unsigned long long>(out_of_bounds_events));
}

}  // namespace openeb_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(openeb_ros2::PreprocessorComponent)
