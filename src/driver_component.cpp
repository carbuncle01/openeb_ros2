#include "openeb_ros2/driver_component.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <ctime>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <system_error>
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

std::string sanitize_label(const std::string & label)
{
  std::string safe;
  safe.reserve(label.size());
  for (const unsigned char c : label) {
    if (std::isalnum(c) || c == '-' || c == '_') {
      safe.push_back(static_cast<char>(c));
    } else if (!safe.empty() && safe.back() != '_') {
      safe.push_back('_');
    }
  }
  while (!safe.empty() && safe.back() == '_') {
    safe.pop_back();
  }
  return safe;
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

diagnostic_msgs::msg::KeyValue make_key_value(
  std::string key, const bool value)
{
  return make_key_value(std::move(key), std::string(value ? "true" : "false"));
}

std::string make_timestamp_string()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#if defined(_WIN32)
  localtime_s(&local_time, &time);
#else
  localtime_r(&time, &local_time);
#endif
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::string format_system_time_utc(
  const std::chrono::system_clock::time_point & time_point)
{
  const auto time = std::chrono::system_clock::to_time_t(time_point);
  std::tm utc_time{};
#if defined(_WIN32)
  gmtime_s(&utc_time, &time);
#else
  gmtime_r(&time, &utc_time);
#endif
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    time_point.time_since_epoch()).count();
  const auto fractional_ns = static_cast<long long>((ns % 1000000000LL + 1000000000LL) %
    1000000000LL);
  std::ostringstream stream;
  stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%S")
         << "." << std::setw(9) << std::setfill('0') << fractional_ns << "Z";
  return stream.str();
}

std::string yaml_quote(const std::string & value)
{
  std::ostringstream stream;
  stream << '"';
  for (const char c : value) {
    if (c == '\\' || c == '"') {
      stream << '\\';
    }
    stream << c;
  }
  stream << '"';
  return stream.str();
}

}  // namespace

DriverComponent::DriverComponent(const rclcpp::NodeOptions & options)
: Node("openeb_driver", options)
{
  serial_ = declare_parameter<std::string>("serial", "");
  device_format_ = declare_parameter<std::string>("device_format", "");
  encoding_ = to_lower(declare_parameter<std::string>("encoding", "evt3"));
  frame_id_ = declare_parameter<std::string>("frame_id", "event_camera");
  raw_recording_enabled_ =
    declare_parameter<bool>("raw_recording_enabled", false);
  raw_recording_request_topic_ =
    declare_parameter<std::string>("raw_recording_request_topic", "raw_recording/request");
  raw_recording_auto_start_ =
    declare_parameter<bool>("raw_recording_auto_start", true);
  raw_recording_dir_ =
    declare_parameter<std::string>("raw_recording_dir", "");
  raw_recording_basename_ =
    declare_parameter<std::string>("raw_recording_basename", "");
  raw_recording_split_duration_s_ =
    declare_parameter<double>("raw_recording_split_duration_s", 0.0);
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
  if (raw_recording_enabled_ && raw_recording_dir_.empty()) {
    throw std::invalid_argument(
      "raw_recording_dir must be set when raw_recording_enabled is true");
  }
  if (raw_recording_split_duration_s_ < 0.0) {
    throw std::invalid_argument(
      "raw_recording_split_duration_s must be non-negative");
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
  if (!raw_recording_request_topic_.empty()) {
    raw_recording_request_sub_ = create_subscription<BagRequest>(
      raw_recording_request_topic_, 10,
      std::bind(
        &DriverComponent::handle_raw_recording_request, this,
        std::placeholders::_1));
  }
  start_raw_recording_service_ = create_service<Trigger>(
    "start_raw_recording",
    std::bind(
      &DriverComponent::handle_start_raw_recording, this,
      std::placeholders::_1, std::placeholders::_2));
  stop_raw_recording_service_ = create_service<Trigger>(
    "stop_raw_recording",
    std::bind(
      &DriverComponent::handle_stop_raw_recording, this,
      std::placeholders::_1, std::placeholders::_2));

  open_camera();

  last_statistics_time_ = std::chrono::steady_clock::now();
  if (statistics_interval_s_ > 0.0) {
    const auto period_ms = std::chrono::milliseconds(
      std::max<std::int64_t>(
        1, static_cast<std::int64_t>(statistics_interval_s_ * 1000.0)));
    statistics_timer_ = create_wall_timer(
      period_ms, std::bind(&DriverComponent::print_statistics, this));
  }

  if (raw_recording_enabled_ && raw_recording_split_duration_s_ > 0.0) {
    const auto period_ms = std::chrono::milliseconds(
      std::max<std::int64_t>(
        1,
        static_cast<std::int64_t>(raw_recording_split_duration_s_ * 1000.0)));
    raw_recording_split_timer_ = create_wall_timer(
      period_ms, std::bind(&DriverComponent::rotate_raw_recording, this));
  }
}

DriverComponent::~DriverComponent()
{
  stop_camera();
}

void DriverComponent::open_camera()
{
  Metavision::DeviceConfig config;
  if (!device_format_.empty()) {
    config.set_format(to_upper(device_format_));
  }

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

  if (raw_recording_enabled_ && raw_recording_auto_start_) {
    start_raw_recording();
  }

  if (!camera_.start()) {
    stop_raw_recording();
    throw std::runtime_error("OpenEB camera did not start");
  }
  camera_running_ = true;

  RCLCPP_INFO(
    get_logger(), "Started OpenEB camera (%ux%u, encoding=%s, device_format=%s, frame_id=%s)",
    width_, height_, encoding_.c_str(),
    device_format_.empty() ? "auto" : device_format_.c_str(),
    frame_id_.c_str());
}

void DriverComponent::stop_camera() noexcept
{
  if (!camera_open_) {
    return;
  }

  try {
    stop_raw_recording();
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

void DriverComponent::start_raw_recording()
{
  std::lock_guard<std::mutex> lock(raw_recording_mutex_);
  if (!raw_recording_enabled_) {
    throw std::runtime_error("raw_recording_enabled is false");
  }
  if (!camera_open_) {
    throw std::runtime_error("camera is not open");
  }
  if (raw_recording_active_) {
    return;
  }
  if (raw_recording_dir_.empty()) {
    throw std::runtime_error("raw_recording_dir is empty");
  }

  std::error_code error_code;
  std::filesystem::create_directories(raw_recording_dir_, error_code);
  if (error_code) {
    throw std::runtime_error(
      "Failed to create raw recording directory '" + raw_recording_dir_ +
      "': " + error_code.message());
  }

  current_raw_recording_path_ = make_raw_recording_path();
  const auto system_start_time = std::chrono::system_clock::now();
  const auto ros_start_time = now();
  if (!camera_.start_recording(current_raw_recording_path_)) {
    throw std::runtime_error(
      "OpenEB raw recording did not start: " +
      current_raw_recording_path_.string());
  }
  write_raw_recording_metadata(
    current_raw_recording_path_, system_start_time, ros_start_time);
  raw_recording_active_ = true;
  RCLCPP_INFO(
    get_logger(), "Started OpenEB raw recording: %s (metadata: %s)",
    current_raw_recording_path_.string().c_str(),
    current_raw_recording_metadata_path_.string().c_str());
}

void DriverComponent::stop_raw_recording() noexcept
{
  std::lock_guard<std::mutex> lock(raw_recording_mutex_);
  if (!raw_recording_active_) {
    return;
  }

  try {
    if (!current_raw_recording_path_.empty()) {
      camera_.stop_recording(current_raw_recording_path_);
    } else {
      camera_.stop_recording();
    }
    RCLCPP_INFO(
      get_logger(), "Stopped OpenEB raw recording: %s",
      current_raw_recording_path_.string().c_str());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      get_logger(), "Failed to stop OpenEB raw recording cleanly: %s",
      error.what());
  }
  raw_recording_active_ = false;
  current_raw_recording_path_.clear();
  current_raw_recording_metadata_path_.clear();
}

void DriverComponent::rotate_raw_recording()
{
  std::lock_guard<std::mutex> lock(raw_recording_mutex_);
  try {
    if (!raw_recording_enabled_ || !camera_open_ || !raw_recording_active_) {
      return;
    }

    const auto previous_path = current_raw_recording_path_;
    const auto next_path = make_raw_recording_path();
    const auto system_start_time = std::chrono::system_clock::now();
    const auto ros_start_time = now();

    if (!camera_.start_recording(next_path)) {
      throw std::runtime_error(
        "OpenEB raw recording did not start: " + next_path.string());
    }

    current_raw_recording_path_ = next_path;
    write_raw_recording_metadata(
      current_raw_recording_path_, system_start_time, ros_start_time);
    if (!previous_path.empty()) {
      try {
        camera_.stop_recording(previous_path);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to stop previous OpenEB raw recording '%s': %s",
          previous_path.string().c_str(), error.what());
      }
    }

    RCLCPP_INFO(
      get_logger(), "Rotated OpenEB raw recording: %s",
      current_raw_recording_path_.string().c_str());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      get_logger(), "Failed to rotate OpenEB raw recording: %s", error.what());
  }
}

std::filesystem::path DriverComponent::make_raw_recording_path()
{
  const auto basename = raw_recording_basename_.empty() ?
    std::string("openeb") :
    raw_recording_basename_;
  const auto serial = serial_.empty() ? std::string("camera") : serial_;

  std::ostringstream filename;
  filename << basename << "_" << serial << "_" << make_timestamp_string();
  if (raw_recording_split_duration_s_ > 0.0) {
    filename << "_" << std::setw(6) << std::setfill('0') << raw_recording_index_;
  }
  filename << ".raw";

  ++raw_recording_index_;
  return std::filesystem::path(raw_recording_dir_) / filename.str();
}

void DriverComponent::write_raw_recording_metadata(
  const std::filesystem::path & raw_path,
  const std::chrono::system_clock::time_point & system_start_time,
  const rclcpp::Time & ros_start_time)
{
  auto metadata_path = raw_path;
  metadata_path += ".metadata.yaml";

  try {
    std::ofstream metadata(metadata_path);
    if (!metadata) {
      throw std::runtime_error("failed to open metadata file");
    }

    const auto system_start_unix_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        system_start_time.time_since_epoch()).count();

    metadata
      << "raw_file: " << yaml_quote(raw_path.string()) << "\n"
      << "recording_start_system_time_utc: "
      << yaml_quote(format_system_time_utc(system_start_time)) << "\n"
      << "recording_start_system_time_unix_ns: " << system_start_unix_ns << "\n"
      << "recording_start_ros_time_sec: " << ros_start_time.seconds() << "\n"
      << "recording_start_ros_time_nanoseconds: " << ros_start_time.nanoseconds() << "\n"
      << "timestamp_reference: "
      << yaml_quote("Metavision RAW event timestamps are relative to this recording start.")
      << "\n"
      << "frame_id: " << yaml_quote(frame_id_) << "\n"
      << "serial: " << yaml_quote(serial_) << "\n"
      << "device_format: "
      << yaml_quote(device_format_.empty() ? std::string("auto") : device_format_) << "\n"
      << "encoding: " << yaml_quote(encoding_) << "\n"
      << "width: " << width_ << "\n"
      << "height: " << height_ << "\n";
    metadata.close();
    if (!metadata) {
      throw std::runtime_error("failed to flush metadata file");
    }
    current_raw_recording_metadata_path_ = metadata_path;
  } catch (const std::exception & error) {
    current_raw_recording_metadata_path_.clear();
    RCLCPP_ERROR(
      get_logger(), "Failed to write OpenEB raw recording metadata '%s': %s",
      metadata_path.string().c_str(), error.what());
  }
}

void DriverComponent::handle_start_raw_recording(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  try {
    start_raw_recording();
    std::string path;
    {
      std::lock_guard<std::mutex> lock(raw_recording_mutex_);
      path = current_raw_recording_path_.string();
    }
    response->success = true;
    response->message = path.empty() ?
      "OpenEB raw recording is already active" :
      "OpenEB raw recording active: " + path;
  } catch (const std::exception & error) {
    response->success = false;
    response->message = error.what();
  }
}

void DriverComponent::handle_stop_raw_recording(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  std::string path;
  bool was_active = false;
  {
    std::lock_guard<std::mutex> lock(raw_recording_mutex_);
    was_active = raw_recording_active_;
    path = current_raw_recording_path_.string();
  }

  stop_raw_recording();
  response->success = true;
  response->message = was_active ?
    "OpenEB raw recording stopped: " + path :
    "OpenEB raw recording was not active";
}

void DriverComponent::handle_raw_recording_request(const BagRequest::SharedPtr request)
{
  if (!request) {
    return;
  }

  try {
    if (request->command == BagRequest::START) {
      const auto safe_label = sanitize_label(request->label);
      {
        std::lock_guard<std::mutex> lock(raw_recording_mutex_);
        if (!raw_recording_active_ && !safe_label.empty()) {
          raw_recording_basename_ = safe_label;
        }
      }
      start_raw_recording();
      RCLCPP_INFO(get_logger(), "OpenEB raw recording START request accepted");
    } else if (request->command == BagRequest::STOP) {
      stop_raw_recording();
      RCLCPP_INFO(get_logger(), "OpenEB raw recording STOP request accepted");
    } else if (request->command == BagRequest::SPLIT) {
      rotate_raw_recording();
      RCLCPP_INFO(get_logger(), "OpenEB raw recording SPLIT request accepted");
    } else if (request->command == BagRequest::MARK) {
      RCLCPP_INFO(
        get_logger(), "OpenEB raw recording MARK request: %s",
        request->label.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "Unknown OpenEB raw recording request command: %u",
        request->command);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      get_logger(), "Failed to handle OpenEB raw recording request: %s",
      error.what());
  }
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

  bool raw_recording_active = false;
  std::string raw_recording_path;
  std::string raw_recording_metadata_path;
  {
    std::lock_guard<std::mutex> lock(raw_recording_mutex_);
    raw_recording_active = raw_recording_active_;
    raw_recording_path = current_raw_recording_path_.string();
    raw_recording_metadata_path = current_raw_recording_metadata_path_.string();
  }

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
      make_key_value("raw_recording_enabled", raw_recording_enabled_),
      make_key_value("raw_recording_request_topic", raw_recording_request_topic_),
      make_key_value("raw_recording_auto_start", raw_recording_auto_start_),
      make_key_value("raw_recording_active", raw_recording_active),
      make_key_value(
        "raw_recording_split_duration_s",
        raw_recording_split_duration_s_),
      make_key_value("raw_recording_path", raw_recording_path),
      make_key_value("raw_recording_metadata_path", raw_recording_metadata_path),
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
      "interarrival_max_us=%.2f no_subscriber=%llu pending_bytes=%llu "
      "raw_recording_active=%s raw_recording_path=%s",
      raw_hz, raw_mib_s, average_raw_kib, publish_hz, publish_mib_s,
      callback_mean_us, callback_max_ns / ns_per_us,
      callback_busy_pct, callback_ns_per_kib, interarrival_mean_us,
      interarrival_max_ns / ns_per_us,
      static_cast<unsigned long long>(no_subscriber),
      static_cast<unsigned long long>(
        pending_bytes_.load(std::memory_order_relaxed)),
      raw_recording_active ? "true" : "false",
      raw_recording_path.c_str());
  }
}

}  // namespace openeb_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(openeb_ros2::DriverComponent)
