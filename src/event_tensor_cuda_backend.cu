#include "openeb_ros2/event_tensor_cuda_backend.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace openeb_ros2
{
namespace
{

void check_cuda(const cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

__global__ void clear_time_slot(
  float * ring, const std::size_t pixels, const std::size_t ring_bins,
  const std::size_t slot, const bool separate, const bool polarity_major)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto planes = separate ? 2U : 1U;
  const auto count = pixels * planes;
  if (index >= count) {
    return;
  }
  const auto polarity = index / pixels;
  const auto pixel = index % pixels;
  const auto channel = separate ?
    (polarity_major ? polarity * ring_bins + slot : slot * 2U + polarity) : slot;
  ring[channel * pixels + pixel] = 0.0F;
}

__global__ void accumulate_events(
  const CudaEvent * events, const std::size_t count, float * ring,
  const std::size_t width, const std::size_t height, const std::size_t ring_bins,
  const std::int64_t origin_timestamp_us, const std::int64_t bin_width_us,
  const std::int64_t oldest_bin,
  const bool separate, const bool polarity_major)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const auto event = events[index];
  if (
    event.timestamp_us < origin_timestamp_us || event.x >= width ||
    event.y >= height)
  {
    return;
  }
  const auto global_bin = (event.timestamp_us - origin_timestamp_us) / bin_width_us;
  if (global_bin < oldest_bin) {
    return;
  }
  const auto slot = static_cast<std::size_t>(
    global_bin % static_cast<std::int64_t>(ring_bins));
  const auto polarity = event.polarity != 0 ? 0U : 1U;
  const auto channel = separate ?
    (polarity_major ? polarity * ring_bins + slot : slot * 2U + polarity) : slot;
  const auto pixel = static_cast<std::size_t>(event.y) * width + event.x;
  const auto value = !separate && event.polarity == 0 ? -1.0F : 1.0F;
  atomicAdd(ring + channel * width * height + pixel, value);
}

__global__ void make_snapshot(
  const float * ring, float * output, const float * means, const float * stddevs,
  const std::size_t pixels, const std::size_t bins, const std::size_t ring_bins,
  const std::size_t channels, const std::int64_t stored_newest_bin,
  const std::int64_t window_end_bin, const bool separate, const bool polarity_major)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto count = channels * pixels;
  if (index >= count) {
    return;
  }
  const auto output_channel = index / pixels;
  const auto pixel = index % pixels;
  std::size_t temporal_bin = output_channel;
  std::size_t polarity = 0U;
  if (separate) {
    if (polarity_major) {
      polarity = output_channel / bins;
      temporal_bin = output_channel % bins;
    } else {
      polarity = output_channel % 2U;
      temporal_bin = output_channel / 2U;
    }
  }
  // window_end_bin is exclusive, matching [window_end - window, window_end).
  const auto global_bin = window_end_bin - static_cast<std::int64_t>(bins) +
    static_cast<std::int64_t>(temporal_bin);
  const auto slot = static_cast<std::size_t>(
    (global_bin % static_cast<std::int64_t>(ring_bins) +
    static_cast<std::int64_t>(ring_bins)) % static_cast<std::int64_t>(ring_bins));
  const auto input_channel = separate ?
    (polarity_major ? polarity * ring_bins + slot : slot * 2U + polarity) : slot;
  const auto stored_oldest_bin =
    stored_newest_bin - static_cast<std::int64_t>(ring_bins - 1U);
  const auto value = global_bin >= stored_oldest_bin && global_bin <= stored_newest_bin ?
    ring[input_channel * pixels + pixel] : 0.0F;
  output[index] = (value - means[output_channel]) /
    stddevs[output_channel];
}

}  // namespace

class EventTensorCudaBackend::Impl
{
public:
  Impl(
    const std::size_t width, const std::size_t height, const std::size_t bins,
    const bool separate, const bool polarity_major, const std::int64_t window_us,
    const std::size_t batch_capacity, const std::vector<double> & channel_mean,
    const std::vector<double> & channel_stddev, const bool enable_event_staging)
  : width_(width), height_(height), bins_(bins), separate_(separate),
    polarity_major_(polarity_major), window_us_(window_us),
    bin_width_us_(window_us / static_cast<std::int64_t>(bins)),
    // Keep one full output window plus one window of look-ahead. Packet decode
    // can deliver events just beyond a scheduled boundary before the timer
    // snapshots that boundary; those events must not overwrite its oldest bin.
    ring_bins_(bins * 2U),
    channels_(bins * (separate ? 2U : 1U)), pixels_(width * height),
    tensor_bytes_(channels_ * pixels_ * sizeof(float)),
    ring_bytes_(ring_bins_ * (separate ? 2U : 1U) * pixels_ * sizeof(float)),
    batch_capacity_(std::max<std::size_t>(1U, batch_capacity))
  {
    if (window_us_ <= 0 || window_us_ % static_cast<std::int64_t>(bins_) != 0) {
      throw std::invalid_argument("CUDA rolling ring requires window_us divisible by bins");
    }
    check_cuda(cudaMalloc(&ring_, ring_bytes_), "cudaMalloc rolling ring");
    check_cuda(cudaMemset(ring_, 0, ring_bytes_), "cudaMemset rolling ring");

    std::vector<float> means(channels_);
    std::vector<float> stddevs(channels_);
    for (std::size_t channel = 0; channel < channels_; ++channel) {
      means[channel] = static_cast<float>(
        channel_mean[channel_mean.size() == 1U ? 0U : channel]);
      stddevs[channel] = static_cast<float>(
        channel_stddev[channel_stddev.size() == 1U ? 0U : channel]);
    }
    check_cuda(cudaMalloc(&means_, channels_ * sizeof(float)), "cudaMalloc channel means");
    check_cuda(cudaMalloc(&stddevs_, channels_ * sizeof(float)), "cudaMalloc channel stddevs");
    check_cuda(
      cudaMemcpy(means_, means.data(), channels_ * sizeof(float), cudaMemcpyHostToDevice),
      "copy channel means");
    check_cuda(
      cudaMemcpy(stddevs_, stddevs.data(), channels_ * sizeof(float), cudaMemcpyHostToDevice),
      "copy channel stddevs");

    if (enable_event_staging) for (auto & slot : transfer_slots_) {
      check_cuda(
        cudaMallocHost(&slot.host, batch_capacity_ * sizeof(CudaEvent)),
        "cudaMallocHost event staging");
      check_cuda(
        cudaMalloc(&slot.device, batch_capacity_ * sizeof(CudaEvent)),
        "cudaMalloc event staging");
      check_cuda(
        cudaEventCreateWithFlags(&slot.complete, cudaEventDisableTiming),
        "cudaEventCreate event staging");
    }
    if (!enable_event_staging) initialize_bin_slots();
  }

  void initialize_bin_slots()
  {
    if (bin_slots_ready_) return;
    for (auto & slot : bin_transfer_slots_) {
      if (slot.host == nullptr) check_cuda(
        cudaMallocHost(&slot.host, pixels_*(separate_ ? 2U:1U)*sizeof(float)), "cudaMallocHost bin staging");
      if (slot.complete == nullptr) check_cuda(
        cudaEventCreateWithFlags(&slot.complete, cudaEventDisableTiming), "cudaEventCreate bin staging");
    }
    bin_slots_ready_ = true;
  }

  ~Impl()
  {
    if (has_stream_work_) {
      (void)cudaStreamSynchronize(last_stream_);
    }
    for (auto & slot : transfer_slots_) {
      if (slot.complete != nullptr) {(void)cudaEventDestroy(slot.complete);}
      if (slot.device != nullptr) {(void)cudaFree(slot.device);}
      if (slot.host != nullptr) {(void)cudaFreeHost(slot.host);}
    }
    for (auto & slot : bin_transfer_slots_) {
      if (slot.complete != nullptr) {(void)cudaEventDestroy(slot.complete);}
      if (slot.host != nullptr) {(void)cudaFreeHost(slot.host);}
    }
    if (stddevs_ != nullptr) {(void)cudaFree(stddevs_);}
    if (means_ != nullptr) {(void)cudaFree(means_);}
    if (ring_ != nullptr) {(void)cudaFree(ring_);}
  }

  void reset(const cudaStream_t stream)
  {
    last_stream_ = stream;
    has_stream_work_ = true;
    check_cuda(cudaMemsetAsync(ring_, 0, ring_bytes_, stream), "reset rolling ring");
    initialized_ = false;
    has_bin_epoch_ = false;
    origin_timestamp_us_ = 0;
    latest_timestamp_us_ = 0;
    accepted_events_ = 0;
    discarded_events_ = 0;
  }

  void update(
    const CudaEvent * const events, const std::size_t event_count,
    const cudaStream_t stream)
  {
    if (events == nullptr || event_count == 0U) {
      return;
    }
    if (has_bin_epoch_ || transfer_slots_[0].host == nullptr)
      throw std::logic_error("Event updates are unavailable for this bin-transfer backend");
    last_stream_ = stream;
    has_stream_work_ = true;
    const auto newest_timestamp = events[event_count - 1U].timestamp_us;
    if (newest_timestamp < 0) {
      discarded_events_ += event_count;
      return;
    }
    if (!initialized_) {
      origin_timestamp_us_ = events[0].timestamp_us;
    }
    const auto newest_bin = (newest_timestamp - origin_timestamp_us_) / bin_width_us_;
    if (!initialized_ || newest_bin < newest_bin_) {
      check_cuda(cudaMemsetAsync(ring_, 0, ring_bytes_, stream), "initialize rolling ring");
      initialized_ = true;
      newest_bin_ = newest_bin;
    } else if (newest_bin > newest_bin_) {
      const auto advance = newest_bin - newest_bin_;
      if (advance >= static_cast<std::int64_t>(ring_bins_)) {
        check_cuda(cudaMemsetAsync(ring_, 0, ring_bytes_, stream), "clear stale rolling ring");
      } else {
        constexpr std::size_t threads = 256U;
        const auto clear_count = pixels_ * (separate_ ? 2U : 1U);
        const auto blocks = static_cast<unsigned int>((clear_count + threads - 1U) / threads);
        for (auto global_bin = newest_bin_ + 1; global_bin <= newest_bin; ++global_bin) {
          const auto slot = static_cast<std::size_t>(
            global_bin % static_cast<std::int64_t>(ring_bins_));
          clear_time_slot<<<blocks, threads, 0, stream>>>(
            ring_, pixels_, ring_bins_, slot, separate_, polarity_major_);
        }
        check_cuda(cudaGetLastError(), "launch rolling slot clear kernel");
      }
      newest_bin_ = newest_bin;
    }

    const auto oldest_bin = newest_bin_ - static_cast<std::int64_t>(ring_bins_ - 1U);
    constexpr std::size_t threads = 256U;
    std::size_t offset = 0U;
    while (offset < event_count) {
      const auto count = std::min(batch_capacity_, event_count - offset);
      auto & slot = transfer_slots_[next_transfer_slot_];
      next_transfer_slot_ = (next_transfer_slot_ + 1U) % transfer_slots_.size();
      if (slot.pending) {
        check_cuda(cudaEventSynchronize(slot.complete), "wait for event staging slot");
      }
      std::memcpy(slot.host, events + offset, count * sizeof(CudaEvent));
      check_cuda(
        cudaMemcpyAsync(
          slot.device, slot.host, count * sizeof(CudaEvent), cudaMemcpyHostToDevice, stream),
        "copy new events to GPU");
      const auto blocks = static_cast<unsigned int>((count + threads - 1U) / threads);
      accumulate_events<<<blocks, threads, 0, stream>>>(
        slot.device, count, ring_, width_, height_, ring_bins_, origin_timestamp_us_,
        bin_width_us_, oldest_bin, separate_, polarity_major_);
      check_cuda(cudaGetLastError(), "launch event accumulation kernel");
      check_cuda(cudaEventRecord(slot.complete, stream), "record event staging completion");
      slot.pending = true;
      offset += count;
    }
    latest_timestamp_us_ = newest_timestamp;
    accepted_events_ += event_count;
  }

  void update_bins(const HistogramBinUpdates & update, const cudaStream_t stream)
  {
    update.validate(width_, height_, bins_, separate_, polarity_major_, bin_width_us_);
    if (initialized_ && !has_bin_epoch_)
      throw std::logic_error("Reset before switching from event updates to bin updates");
    const bool new_epoch = !has_bin_epoch_ || update.epoch != bin_epoch_;
    if (!new_epoch && (update.origin_us != origin_timestamp_us_ ||
      update.newest_bin < newest_bin_ || update.latest_us < latest_timestamp_us_))
      throw std::invalid_argument("Out-of-order absolute bin update");
    last_stream_ = stream;
    has_stream_work_ = true;
    try {
      initialize_bin_slots();
      if (new_epoch) check_cuda(cudaMemsetAsync(ring_, 0, ring_bytes_, stream), "reset bin-transfer ring");
      const auto planes = separate_ ? 2U:1U;
      for (std::size_t i=0; i<update.slots.size(); ++i) {
        auto & staging = bin_transfer_slots_[next_bin_transfer_slot_];
        next_bin_transfer_slot_ = (next_bin_transfer_slot_+1U)%bin_transfer_slots_.size();
        if (staging.pending) check_cuda(cudaEventSynchronize(staging.complete), "wait for bin staging");
        std::memcpy(staging.host, update.values.data()+i*planes*pixels_, planes*pixels_*sizeof(float));
        for (std::size_t p=0; p<planes; ++p) {
          const auto channel = separate_ ?
            (polarity_major_ ? p*ring_bins_+update.slots[i] : update.slots[i]*2U+p) : update.slots[i];
          check_cuda(cudaMemcpyAsync(ring_+channel*pixels_, staging.host+p*pixels_,
            pixels_*sizeof(float), cudaMemcpyHostToDevice, stream), "upload dirty histogram plane");
        }
        check_cuda(cudaEventRecord(staging.complete, stream), "record bin staging completion");
        staging.pending = true;
      }
      origin_timestamp_us_ = update.origin_us;
      newest_bin_ = update.newest_bin; latest_timestamp_us_ = update.latest_us;
      bin_epoch_ = update.epoch; has_bin_epoch_ = true; initialized_ = true;
    } catch (...) {
      // An upload may have enqueued one plane before failing. Its pinned host
      // storage must survive until pending DMA is done, even on the error path.
      (void)cudaStreamSynchronize(stream);
      initialized_ = false; has_bin_epoch_ = false;
      throw;
    }
  }

  void snapshot(
    float * output, const std::int64_t window_end_us, const cudaStream_t stream)
  {
    if (!initialized_) {
      throw std::runtime_error("CUDA rolling ring is not initialized");
    }
    const auto elapsed_us = window_end_us - origin_timestamp_us_;
    if (elapsed_us < window_us_ || elapsed_us % bin_width_us_ != 0) {
      throw std::invalid_argument("CUDA snapshot end is not aligned to the event-bin clock");
    }
    const auto window_end_bin = elapsed_us / bin_width_us_;
    last_stream_ = stream;
    has_stream_work_ = true;
    constexpr std::size_t threads = 256U;
    const auto count = channels_ * pixels_;
    const auto blocks = static_cast<unsigned int>((count + threads - 1U) / threads);
    make_snapshot<<<blocks, threads, 0, stream>>>(
      ring_, output, means_, stddevs_, pixels_, bins_, ring_bins_, channels_, newest_bin_,
      window_end_bin, separate_, polarity_major_);
    check_cuda(cudaGetLastError(), "launch event snapshot kernel");
  }

  bool ready() const
  {
    return initialized_;
  }

  std::int64_t latest_window_end_us() const
  {
    return initialized_ && newest_bin_ + 1 >= static_cast<std::int64_t>(bins_) ?
      origin_timestamp_us_ + (newest_bin_ + 1) * bin_width_us_ : 0;
  }

  struct TransferSlot
  {
    CudaEvent * host{nullptr};
    CudaEvent * device{nullptr};
    cudaEvent_t complete{nullptr};
    bool pending{false};
  };

  struct BinTransferSlot {
    float * host{nullptr};
    cudaEvent_t complete{nullptr};
    bool pending{false};
  };
  std::vector<BinTransferSlot> bin_transfer_slots_{4};
  std::size_t next_bin_transfer_slot_{0};
  bool bin_slots_ready_{false}, has_bin_epoch_{false};
  std::uint64_t bin_epoch_{0};
  std::size_t width_;
  std::size_t height_;
  std::size_t bins_;
  bool separate_;
  bool polarity_major_;
  std::int64_t window_us_;
  std::int64_t bin_width_us_;
  std::size_t ring_bins_;
  std::size_t channels_;
  std::size_t pixels_;
  std::size_t tensor_bytes_;
  std::size_t ring_bytes_;
  std::size_t batch_capacity_;
  float * ring_{nullptr};
  float * means_{nullptr};
  float * stddevs_{nullptr};
  std::vector<TransferSlot> transfer_slots_{4};
  std::size_t next_transfer_slot_{0};
  cudaStream_t last_stream_{nullptr};
  bool has_stream_work_{false};
  bool initialized_{false};
  std::int64_t origin_timestamp_us_{0};
  std::int64_t newest_bin_{0};
  std::int64_t latest_timestamp_us_{0};
  std::uint64_t accepted_events_{0};
  std::uint64_t discarded_events_{0};
};

EventTensorCudaBackend::EventTensorCudaBackend(
  const std::size_t width, const std::size_t height, const std::size_t bins,
  const bool separate_polarities, const bool polarity_major,
  const std::int64_t window_us, const std::size_t batch_capacity,
  const std::vector<double> & channel_mean, const std::vector<double> & channel_stddev,
  const bool enable_event_staging)
: impl_(std::make_unique<Impl>(
      width, height, bins, separate_polarities, polarity_major, window_us,
      batch_capacity, channel_mean, channel_stddev, enable_event_staging))
{
}

EventTensorCudaBackend::~EventTensorCudaBackend() = default;

void EventTensorCudaBackend::reset(const cudaStream_t stream) {impl_->reset(stream);}

void EventTensorCudaBackend::update(
  const std::vector<CudaEvent> & events, const cudaStream_t stream)
{
  impl_->update(events.data(), events.size(), stream);
}

void EventTensorCudaBackend::update(
  const CudaEvent * const events, const std::size_t count, const cudaStream_t stream)
{
  impl_->update(events, count, stream);
}

void EventTensorCudaBackend::update_bins(
  const HistogramBinUpdates & updates, const cudaStream_t stream)
{
  impl_->update_bins(updates, stream);
}

void EventTensorCudaBackend::snapshot(
  float * output, const std::int64_t window_end_us, const cudaStream_t stream)
{
  impl_->snapshot(output, window_end_us, stream);
}

bool EventTensorCudaBackend::ready() const {return impl_->ready();}
std::int64_t EventTensorCudaBackend::latest_timestamp_us() const
{
  return impl_->latest_timestamp_us_;
}
std::int64_t EventTensorCudaBackend::latest_window_end_us() const
{
  return impl_->latest_window_end_us();
}
std::uint64_t EventTensorCudaBackend::accepted_events() const {return impl_->accepted_events_;}
std::uint64_t EventTensorCudaBackend::discarded_events() const {return impl_->discarded_events_;}

}  // namespace openeb_ros2
