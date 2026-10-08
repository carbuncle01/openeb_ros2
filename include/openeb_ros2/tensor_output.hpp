#ifndef OPENEB_ROS2__TENSOR_OUTPUT_HPP_
#define OPENEB_ROS2__TENSOR_OUTPUT_HPP_
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>
#include "cuda_buffer/cuda_buffer_api.hpp"
#include "isaac_ros_tensor_msgs/msg/tensor_list.hpp"
#include "tensor_msgs/msg/experimental_tensor.hpp"
#include "std_msgs/msg/header.hpp"
namespace openeb_ros2 {
inline auto allocate_tensor(tensor_msgs::msg::ExperimentalTensor & tensor,
  const std::vector<std::int64_t> & shape, cudaStream_t stream)
{
  std::size_t bytes = sizeof(float);
  for (auto dimension : shape) {
    if (dimension <= 0 || static_cast<std::uint64_t>(dimension) > SIZE_MAX / bytes) {
      throw std::invalid_argument("Invalid or overflowing tensor shape");
    }
    bytes *= static_cast<std::size_t>(dimension);
  }
  tensor.shape.assign(shape.begin(), shape.end());
  tensor.dtype_code = 2; // DLPack kDLFloat
  tensor.dtype_bits = 32;
  tensor.dtype_lanes = 1;
  tensor.byte_offset = 0;
  tensor.strides.clear(); // contiguous NCHW
  tensor.data = cuda_buffer_backend::allocate_buffer(bytes);
  return cuda_buffer_backend::from_output_buffer(tensor.data, stream);
}
inline isaac_ros_tensor_msgs::msg::TensorList make_tensor_list(
  const std_msgs::msg::Header & header, const std::string & name,
  tensor_msgs::msg::ExperimentalTensor tensor)
{
  isaac_ros_tensor_msgs::msg::TensorList message;
  message.header = header;
  message.names.push_back(name);
  message.tensors.push_back(std::move(tensor));
  return message;
}
} // namespace openeb_ros2
#endif
