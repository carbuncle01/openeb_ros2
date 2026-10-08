#include "openeb_ros2/event_tensor_cuda_backend.hpp"

#include <cuda_runtime_api.h>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

namespace
{

bool expect_value(
  const std::vector<float> & tensor, const std::size_t channel,
  const std::size_t pixel, const float expected)
{
  constexpr std::size_t kPixels = 12U;
  const auto actual = tensor[channel * kPixels + pixel];
  if (std::fabs(actual - expected) <= 1.0e-6F) {
    return true;
  }
  std::cerr << "channel=" << channel << " pixel=" << pixel <<
    " expected=" << expected << " actual=" << actual << '\n';
  return false;
}

}  // namespace

int main()
{
  constexpr std::size_t kWidth = 4U;
  constexpr std::size_t kHeight = 3U;
  constexpr std::size_t kBins = 2U;
  constexpr std::size_t kChannels = 4U;
  constexpr std::size_t kElements = kChannels * kWidth * kHeight;

  cudaStream_t stream{nullptr};
  if (cudaStreamCreate(&stream) != cudaSuccess) {
    return 1;
  }
  float * device_output{nullptr};
  if (
    cudaMalloc(reinterpret_cast<void **>(&device_output), kElements * sizeof(float)) !=
    cudaSuccess)
  {
    (void)cudaStreamDestroy(stream);
    return 1;
  }

  bool ok = true;
  {
    openeb_ros2::EventTensorCudaBackend backend(
      kWidth, kHeight, kBins, true, true, 2000, 16,
      std::vector<double>{0.0}, std::vector<double>{1.0});

  // Origin is 1000 us. The 3000 us event belongs to the following half-open
  // window and must not overwrite the oldest bin needed by [1000, 3000).
  const std::vector<openeb_ros2::CudaEvent> events{
      {1000, 1, 1, 1, {0, 0, 0}},
      {1500, 2, 1, 0, {0, 0, 0}},
      {2000, 3, 2, 1, {0, 0, 0}},
      {3000, 0, 0, 0, {0, 0, 0}},
  };
  // The asynchronous preprocessor sends bounded slices of a decoded packet.
  backend.update(events.data(), events.size(), stream);

  std::vector<float> output(kElements, 0.0F);
  backend.snapshot(device_output, 3000, stream);
  if (cudaMemcpy(
      output.data(), device_output, kElements * sizeof(float),
      cudaMemcpyDeviceToHost) != cudaSuccess)
  {
    return 1;
  }
  ok &= expect_value(output, 0, 1U * kWidth + 1U, 1.0F);
  ok &= expect_value(output, 1, 2U * kWidth + 3U, 1.0F);
  ok &= expect_value(output, 2, 1U * kWidth + 2U, 1.0F);
  ok &= expect_value(output, 3, 0U, 0.0F);

  // Advancing the scheduled window without new events must decay old bins and
  // expose only the event in [3000, 5000), rather than replaying a stale tensor.
  backend.snapshot(device_output, 5000, stream);
  if (cudaMemcpy(
      output.data(), device_output, kElements * sizeof(float),
      cudaMemcpyDeviceToHost) != cudaSuccess)
  {
    return 1;
  }
  ok &= expect_value(output, 0, 0U, 0.0F);
  ok &= expect_value(output, 1, 0U, 0.0F);
  ok &= expect_value(output, 2, 0U, 1.0F);
  ok &= expect_value(output, 3, 0U, 0.0F);
  }

  (void)cudaFree(device_output);
  (void)cudaStreamDestroy(stream);
  return ok ? 0 : 1;
}
