#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "openeb_ros2/driver_component.hpp"
#include "openeb_ros2/preprocessor_component.hpp"
#include "openeb_ros2/event_tensor_encoder_node.hpp"
#include "openeb_ros2/async_event_tensor_preprocessor_node.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
    auto selector = std::make_shared<rclcpp::Node>("tensor_pipeline", options);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Tensor representation backend: cpu, cuda, cuda_async; restart to change";
    const auto backend = selector->declare_parameter<std::string>("tensor_backend", "cuda_async", descriptor);
    if (backend != "cpu" && backend != "cuda" && backend != "cuda_async") {
      throw std::invalid_argument("tensor_backend must be cpu, cuda, or cuda_async");
    }
    descriptor.description = "Optional async aggregation override; empty preserves tensor YAML";
    const auto aggregation = selector->declare_parameter<std::string>("event_aggregation", "", descriptor);
    if (!aggregation.empty() && (backend != "cuda_async" ||
      (aggregation != "events" && aggregation != "evt3_runs" && aggregation != "evt3_runs_gpu" &&
       aggregation != "evt3_runs_gpu_direct" && aggregation != "evt3_runs_grouped_gpu")))
      throw std::invalid_argument("event_aggregation override requires cuda_async and a valid aggregation mode");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    const auto ns = std::string(selector->get_namespace());
    auto tensor_options = options;
    tensor_options.arguments({"--ros-args", "-r", "__node:=event_tensor", "-r", "__ns:=" + ns});
    tensor_options.append_parameter_override("subscribe_packets", false);
    if (!aggregation.empty()) tensor_options.append_parameter_override("event_aggregation", aggregation);
    std::shared_ptr<rclcpp::Node> tensor;
    openeb_ros2::DriverComponent::PacketSink sink;
    if (backend == "cuda_async") {
      auto node = std::make_shared<openeb_ros2::AsyncEventTensorPreprocessorNode>(tensor_options);
      sink = [node](auto packet) { node->ingest(std::move(packet)); };
      tensor = node;
    } else {
      tensor_options.append_parameter_override("representation_backend", backend);
      auto node = std::make_shared<openeb_ros2::EventTensorEncoderNode>(tensor_options);
      sink = [node](auto packet) { node->ingest(std::move(packet)); };
      tensor = node;
    }
    auto image_options = options;
    image_options.arguments({"--ros-args", "-r", "__node:=event_preprocessor", "-r", "__ns:=" + ns});
    image_options.append_parameter_override("subscribe_packets", false);
    image_options.append_parameter_override("packet_forward_enabled", false);
    auto image = std::make_shared<openeb_ros2::PreprocessorComponent>(image_options);
    auto tensor_sink = std::move(sink);
    sink = [image, tensor_sink](auto packet) {
      image->enqueue_image(*packet);
      tensor_sink(std::move(packet));
    };
    auto driver_options = options;
    driver_options.arguments({"--ros-args", "-r", "__node:=event_camera_driver", "-r", "__ns:=" + ns});
    auto driver = std::make_shared<openeb_ros2::DriverComponent>(driver_options, std::move(sink));
    // TensorRT components can be loaded into this process through the manager's services.
    auto manager_options = options;
    manager_options.arguments({"--ros-args", "-r", "__node:=tensor_container", "-r", "__ns:=" + ns});
    auto executor_alias = std::shared_ptr<rclcpp::Executor>(&executor, [](auto *) {});
    auto manager = std::make_shared<rclcpp_components::ComponentManager>(executor_alias, "tensor_container", manager_options);
    executor.add_node(selector);
    executor.add_node(tensor);
    executor.add_node(driver);
    executor.add_node(image);
    executor.add_node(manager);
    RCLCPP_INFO(selector->get_logger(), "Direct OpenEB -> %s -> GPU FP32 NCHW TensorList; container=%s",
      backend.c_str(), manager->get_fully_qualified_name());
    executor.spin();
    executor.remove_node(driver);
    driver.reset(); // Stop/join SDK callbacks before destroying tensor state.
    executor.remove_node(image);
    image.reset();
    executor.remove_node(tensor);
    executor.remove_node(manager);
    executor.remove_node(selector);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("openeb_tensor_pipeline"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
