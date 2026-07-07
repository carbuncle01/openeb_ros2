#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "openeb_ros2/preprocessor_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<openeb_ros2::PreprocessorComponent>(
    rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
