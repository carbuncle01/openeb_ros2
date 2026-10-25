#ifndef OPENEB_ROS2__EVENT_TENSOR_CUDA_BACKEND_HPP_
#define OPENEB_ROS2__EVENT_TENSOR_CUDA_BACKEND_HPP_

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>
#include "openeb_ros2/histogram_bin_updates.hpp"

namespace openeb_ros2
{

struct alignas(16) CudaEvent
{
  std::int64_t timestamp_us;
  std::uint16_t x;
  std::uint16_t y;
  std::uint8_t polarity;
  std::uint8_t padding[3]{};
};

class EventTensorCudaBackend
{
public:
  EventTensorCudaBackend(
    std::size_t width, std::size_t height, std::size_t bins,
    bool separate_polarities, bool polarity_major,
    std::int64_t window_us, std::size_t batch_capacity,
    const std::vector<double> & channel_mean,
    const std::vector<double> & channel_stddev,
    bool enable_event_staging = true);
  ~EventTensorCudaBackend();

  EventTensorCudaBackend(const EventTensorCudaBackend &) = delete;
  EventTensorCudaBackend & operator=(const EventTensorCudaBackend &) = delete;

  void reset(cudaStream_t stream);
  void update(const std::vector<CudaEvent> & events, cudaStream_t stream);
  void update(const CudaEvent * events, std::size_t count, cudaStream_t stream);
  // Copies absolute dirty CPU bins through owned pinned buffers. Calls must
  // use one stream; do not mix event updates and bin updates without reset().
  void update_bins(const HistogramBinUpdates & updates, cudaStream_t stream);
  void snapshot(float * output, std::int64_t window_end_us, cudaStream_t stream);

  bool ready() const;
  std::int64_t latest_timestamp_us() const;
  std::int64_t latest_window_end_us() const;
  std::uint64_t accepted_events() const;
  std::uint64_t discarded_events() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace openeb_ros2

#endif  // OPENEB_ROS2__EVENT_TENSOR_CUDA_BACKEND_HPP_
