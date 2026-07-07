#include "openeb_ros2/driver_component.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <functional>
#include <stdexcept>
#include <utility>

#include <metavision/hal/utils/device_config.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace openeb_ros2
{
namespace
{

std::string to_lower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string to_upper(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return value;
}

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

diagnostic_msgs::msg::KeyValue make_key_value(
  std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue diagnostic_value;
  diagnostic_value.key = std::move(key);
  diagnostic_value.value = std::move(value);
  return diagnostic_value;
}

diagnostic_msgs::msg::KeyValue make_key_value(
  std::string key, const double value)
{
  return make_key_value(std::move(key), std::to_string(value));
}

diagnostic_msgs::msg::KeyValue make_key_value(
  std::string key, const std::uint64_t value)
{
  return make_key_value(std::move(key), std::to_string(value));
}

}  // namespace

DriverComponent::DriverComponent(const rclcpp::NodeOptions & options)
: Node("openeb_driver", options)
{
  serial_ = declare_parameter<std::string>("serial", "");
  encoding_ = to_lower(declare_parameter<std::string>("encoding", "evt3"));
  frame_id_ = declare_parameter<std::string>("frame_id", "event_camera");
  packet_duration_us_ = declare_parameter<std::int64_t>("packet_duration_us", 1000);

  const auto packet_size =
    declare_parameter<std::int64_t>("packet_size_bytes", 1000000);
  const auto publisher_depth =
    declare_parameter<std::int64_t>("publisher_depth", 8);
  statistics_interval_s_ =
    declare_parameter<double>("statistics_interval_s", 1.0);
  debug_ = declare_parameter<bool>("debug", false);

  if (encoding_ != "evt3") {
    throw std::invalid_argument("Only the evt3 encoding is currently supported");
  }
  if (packet_duration_us_ < 0) {
    throw std::invalid_argument("packet_duration_us must be non-negative");
  }
  if (packet_size <= 0) {
    throw std::invalid_argument("packet_size_bytes must be positive");
  }
  if (publisher_depth <= 0) {
    throw std::invalid_argument("publisher_depth must be positive");
  }
  if (statistics_interval_s_ < 0.0) {
    throw std::invalid_argument("statistics_interval_s must be non-negative");
  }

  packet_size_bytes_ = static_cast<std::size_t>(packet_size);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(
      static_cast<std::size_t>(publisher_depth)))
                     .best_effort()
                     .durability_volatile();
  event_publisher_ = create_publisher<EventPacket>("events_raw", qos);
  const auto diagnostics_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                 .reliable()
                                 .durability_volatile();
  diagnostics_publisher_ =
    create_publisher<DiagnosticArray>("diagnostics", diagnostics_qos);

  open_camera();

  last_statistics_time_ = std::chrono::steady_clock::now();
  if (statistics_interval_s_ > 0.0) {
    const auto period_ms = std::chrono::milliseconds(
      std::max<std::int64_t>(
        1, static_cast<std::int64_t>(statistics_interval_s_ * 1000.0)));
    statistics_timer_ = create_wall_timer(
      period_ms, std::bind(&DriverComponent::print_statistics, this));
  }
}

DriverComponent::~DriverComponent()
{
  stop_camera();
}

void DriverComponent::open_camera()
{
  Metavision::DeviceConfig config;
  config.set_format(to_upper(encoding_));

  camera_ = serial_.empty() ?
    Metavision::Camera::from_first_available(config) :
    Metavision::Camera::from_serial(serial_, config);
  camera_open_ = true;

  const auto & geometry = camera_.geometry();
  width_ = static_cast<std::uint32_t>(geometry.get_width());
  height_ = static_cast<std::uint32_t>(geometry.get_height());

  raw_callback_id_ = camera_.raw_data().add_callback(
    [this](const std::uint8_t * data, const std::size_t size) {
      on_raw_data(data, size);
    });
  raw_callback_active_ = true;

  runtime_error_callback_id_ = camera_.add_runtime_error_callback(
    [this](const Metavision::CameraException & error) {
      RCLCPP_ERROR(get_logger(), "OpenEB camera error: %s", error.what());
    });
  runtime_error_callback_active_ = true;

  if (!camera_.start()) {
    throw std::runtime_error("OpenEB camera did not start");
  }
  camera_running_ = true;

  RCLCPP_INFO(
    get_logger(), "Started OpenEB camera (%ux%u, encoding=%s, frame_id=%s)",
    width_, height_, encoding_.c_str(), frame_id_.c_str());
}

void DriverComponent::stop_camera() noexcept
{
  if (!camera_open_) {
    return;
  }

  try {
    if (camera_running_) {
      camera_.stop();
      camera_running_ = false;
    }
    if (raw_callback_active_) {
      camera_.raw_data().remove_callback(raw_callback_id_);
      raw_callback_active_ = false;
    }
    if (runtime_error_callback_active_) {
      camera_.remove_runtime_error_callback(runtime_error_callback_id_);
      runtime_error_callback_active_ = false;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to stop OpenEB camera cleanly: %s", error.what());
  }

  pending_packet_.reset();
  camera_open_ = false;
}

void DriverComponent::on_raw_data(const std::uint8_t * data, const std::size_t size)
{
  if (data == nullptr || size == 0) {
    return;
  }

  const auto callback_start = std::chrono::steady_clock::now();

  if (!has_output_subscribers()) {
    no_subscriber_callbacks_.fetch_add(1, std::memory_order_relaxed);
    pending_packet_.reset();
    pending_bytes_.store(0, std::memory_order_relaxed);
    record_raw_callback(callback_start, size);
    return;
  }

  const auto steady_now = callback_start;
  if (!pending_packet_) {
    pending_packet_ = std::make_unique<EventPacket>();
    pending_packet_->header.stamp = now();
    pending_packet_->header.frame_id = frame_id_;
    pending_packet_->time_base = 0;
    pending_packet_->encoding = encoding_;
    pending_packet_->seq = sequence_++;
    pending_packet_->width = width_;
    pending_packet_->height = height_;
    pending_packet_->events.reserve(reserve_size_);
  }

  pending_packet_->events.insert(
    pending_packet_->events.end(), data, data + size);
  pending_bytes_.store(
    pending_packet_->events.size(), std::memory_order_relaxed);

  const bool duration_reached =
    packet_duration_us_ == 0 || !has_published_ ||
    std::chrono::duration_cast<std::chrono::microseconds>(
      steady_now - last_publish_time_).count() >= packet_duration_us_;
  const bool size_reached =
    pending_packet_->events.size() >= packet_size_bytes_;

  if (duration_reached || size_reached) {
    publish_pending_packet(steady_now);
  }

  record_raw_callback(callback_start, size);
}

void DriverComponent::publish_pending_packet(
  const std::chrono::steady_clock::time_point & publish_time)
{
  if (!pending_packet_) {
    return;
  }

  const auto packet_bytes = pending_packet_->events.size();
  reserve_size_ = std::max(reserve_size_, packet_bytes);
  published_messages_.fetch_add(1, std::memory_order_relaxed);
  published_bytes_.fetch_add(packet_bytes, std::memory_order_relaxed);
  event_publisher_->publish(std::move(pending_packet_));
  pending_bytes_.store(0, std::memory_order_relaxed);
  last_publish_time_ = publish_time;
  has_published_ = true;
}

bool DriverComponent::has_output_subscribers() const
{
  return event_publisher_->get_subscription_count() > 0 ||
         event_publisher_->get_intra_process_subscription_count() > 0;
}

void DriverComponent::record_raw_callback(
  const std::chrono::steady_clock::time_point & callback_start,
  const std::size_t bytes)
{
  const auto callback_end = std::chrono::steady_clock::now();
  const auto callback_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      callback_end - callback_start).count());

  raw_callbacks_.fetch_add(1, std::memory_order_relaxed);
  raw_bytes_.fetch_add(bytes, std::memory_order_relaxed);
  callback_time_ns_.fetch_add(callback_ns, std::memory_order_relaxed);
  update_max(callback_time_max_ns_, callback_ns);

  const auto arrival_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    callback_start.time_since_epoch()).count();
  const auto previous_ns =
    last_raw_arrival_ns_.exchange(arrival_ns, std::memory_order_relaxed);
  if (previous_ns > 0 && arrival_ns > previous_ns) {
    const auto interval_ns =
      static_cast<std::uint64_t>(arrival_ns - previous_ns);
    interarrival_time_ns_.fetch_add(interval_ns, std::memory_order_relaxed);
    interarrival_samples_.fetch_add(1, std::memory_order_relaxed);
    update_max(interarrival_time_max_ns_, interval_ns);
  }
}

void DriverComponent::print_statistics()
{
  const auto statistics_time = std::chrono::steady_clock::now();
  const auto elapsed_s =
    std::chrono::duration<double>(statistics_time - last_statistics_time_).count();
  last_statistics_time_ = statistics_time;
  if (elapsed_s <= 0.0) {
    return;
  }

  const auto raw_callbacks =
    raw_callbacks_.exchange(0, std::memory_order_relaxed);
  const auto raw_bytes =
    raw_bytes_.exchange(0, std::memory_order_relaxed);
  const auto published_messages =
    published_messages_.exchange(0, std::memory_order_relaxed);
  const auto published_bytes =
    published_bytes_.exchange(0, std::memory_order_relaxed);
  const auto no_subscriber =
    no_subscriber_callbacks_.exchange(0, std::memory_order_relaxed);
  const auto callback_ns =
    callback_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto callback_max_ns =
    callback_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto interarrival_ns =
    interarrival_time_ns_.exchange(0, std::memory_order_relaxed);
  const auto interarrival_max_ns =
    interarrival_time_max_ns_.exchange(0, std::memory_order_relaxed);
  const auto interarrival_samples =
    interarrival_samples_.exchange(0, std::memory_order_relaxed);

  constexpr double bytes_per_mib = 1024.0 * 1024.0;
  constexpr double bytes_per_kib = 1024.0;
  constexpr double ns_per_us = 1000.0;

  const double raw_hz = raw_callbacks / elapsed_s;
  const double raw_mib_s = raw_bytes / bytes_per_mib / elapsed_s;
  const double average_raw_kib =
    raw_callbacks == 0 ? 0.0 :
    static_cast<double>(raw_bytes) / raw_callbacks / bytes_per_kib;
  const double publish_hz = published_messages / elapsed_s;
  const double publish_mib_s = published_bytes / bytes_per_mib / elapsed_s;
  const double callback_mean_us =
    raw_callbacks == 0 ? 0.0 :
    static_cast<double>(callback_ns) / raw_callbacks / ns_per_us;
  const double callback_busy_pct =
    static_cast<double>(callback_ns) / (elapsed_s * 1.0e9) * 100.0;
  const double callback_ns_per_kib =
    raw_bytes == 0 ? 0.0 :
    static_cast<double>(callback_ns) /
    (static_cast<double>(raw_bytes) / bytes_per_kib);
  const double interarrival_mean_us =
    interarrival_samples == 0 ? 0.0 :
    static_cast<double>(interarrival_ns) / interarrival_samples / ns_per_us;

  const bool publish_diagnostics =
    diagnostics_publisher_->get_subscription_count() > 0 ||
    diagnostics_publisher_->get_intra_process_subscription_count() > 0;
  if (publish_diagnostics) {
    DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = std::string(get_fully_qualified_name()) + ": driver_stats";
    status.message = "OK";
    status.hardware_id = serial_.empty() ? "openeb_camera" : serial_;
    status.values = {
      make_key_value("raw_hz", raw_hz),
      make_key_value("raw_mib_s", raw_mib_s),
      make_key_value("avg_raw_kib", average_raw_kib),
      make_key_value("publish_hz", publish_hz),
      make_key_value("publish_mib_s", publish_mib_s),
      make_key_value("callback_mean_us", callback_mean_us),
      make_key_value("callback_max_us", callback_max_ns / ns_per_us),
      make_key_value("callback_busy_pct", callback_busy_pct),
      make_key_value("callback_ns_per_kib", callback_ns_per_kib),
      make_key_value("interarrival_mean_us", interarrival_mean_us),
      make_key_value("interarrival_max_us", interarrival_max_ns / ns_per_us),
      make_key_value("no_subscriber", no_subscriber),
      make_key_value(
        "pending_bytes", pending_bytes_.load(std::memory_order_relaxed)),
    };
    diagnostics.status.push_back(std::move(status));
    diagnostics_publisher_->publish(std::move(diagnostics));
  }

  if (debug_) {
    RCLCPP_INFO(
      get_logger(),
      "driver_stats raw_hz=%.1f raw_mib_s=%.3f avg_raw_kib=%.2f "
      "publish_hz=%.1f publish_mib_s=%.3f callback_mean_us=%.2f "
      "callback_max_us=%.2f callback_busy_pct=%.2f "
      "callback_ns_per_kib=%.2f interarrival_mean_us=%.2f "
      "interarrival_max_us=%.2f no_subscriber=%llu pending_bytes=%llu",
      raw_hz, raw_mib_s, average_raw_kib, publish_hz, publish_mib_s,
      callback_mean_us, callback_max_ns / ns_per_us,
      callback_busy_pct, callback_ns_per_kib, interarrival_mean_us,
      interarrival_max_ns / ns_per_us,
      static_cast<unsigned long long>(no_subscriber),
      static_cast<unsigned long long>(
        pending_bytes_.load(std::memory_order_relaxed)));
  }
}

}  // namespace openeb_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(openeb_ros2::DriverComponent)
