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

// One warp per run; no expanded event array is allocated on CPU or GPU.
__global__ void accumulate_evt3_runs(
  const std::uint16_t * words, const Evt3Run * runs, std::size_t run_count,
  float * ring, std::size_t width, std::size_t height, unsigned input_width,
  std::size_t ring_bins, std::int64_t oldest, bool separate, bool major)
{
  const auto thread=static_cast<std::size_t>(blockIdx.x)*blockDim.x+threadIdx.x;
  const auto run_index=thread/32;const unsigned lane=thread%32;
  if(run_index>=run_count)return;
  const auto run=runs[run_index];if(run.bin<oldest)return;
  const auto slot=static_cast<std::size_t>(run.bin%static_cast<std::int64_t>(ring_bins));
  const unsigned count=run.kind==2?run.word_count:(run.kind==4?12u:8u);
  for(unsigned i=lane;i<count;i+=32) {
    unsigned x,polarity;
    if(run.kind==2){const auto word=words[run.word_offset+i];x=word&2047;polarity=(word>>11)&1;}
    else {
      if(!(words[run.word_offset]&(1u<<i)))continue;
      x=static_cast<std::uint16_t>(run.base_x+i);polarity=run.polarity;
    }
    if(x>=input_width)continue;
    const auto p=polarity?0u:1u;
    const auto channel=separate?(major?p*ring_bins+slot:slot*2+p):slot;
    const auto pixel=static_cast<std::size_t>(run.y)*width+static_cast<std::size_t>(x)*width/input_width;
    atomicAdd(ring+channel*width*height+pixel,!separate&&!polarity?-1.0F:1.0F);
  }
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
    for(auto & slot : grouped_slots_) {
      if(slot.complete)(void)cudaEventDestroy(slot.complete);
      if(slot.host_raw)(void)cudaFreeHost(slot.host_raw);
      if(slot.host_runs)(void)cudaFreeHost(slot.host_runs);
      if(slot.device_raw)(void)cudaFree(slot.device_raw);
      if(slot.device_runs)(void)cudaFree(slot.device_runs);
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
    initialized_ = false;
    check_cuda(cudaMemsetAsync(ring_, 0, ring_bytes_, stream), "reset rolling ring");
    has_bin_epoch_ = false;
    has_grouped_epoch_ = false;
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
    if (has_bin_epoch_ || has_grouped_epoch_ || transfer_slots_[0].host == nullptr)
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

  void update_bins_direct(const HistogramBinView & update, const cudaStream_t stream)
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
        update.copy_slot_to(i, static_cast<float *>(staging.host));
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

  void reserve_grouped_raw(std::size_t raw_bytes, std::size_t runs)
  {
    if(!raw_bytes || !runs || runs>SIZE_MAX/sizeof(Evt3Run))throw std::invalid_argument("Grouped staging capacity must be positive");
    for(auto& slot:grouped_slots_) {
      if(slot.pending) {check_cuda(cudaEventSynchronize(slot.complete),"wait for grouped staging resize");slot.pending=false;}
      if(!slot.complete)check_cuda(cudaEventCreateWithFlags(&slot.complete,cudaEventDisableTiming),"create grouped staging event");
      if(raw_bytes>slot.raw_capacity) {
        if(slot.host_raw){check_cuda(cudaFreeHost(slot.host_raw),"free grouped host RAW");slot.host_raw=nullptr;}
        if(slot.device_raw){check_cuda(cudaFree(slot.device_raw),"free grouped device RAW");slot.device_raw=nullptr;}
        slot.raw_capacity=0;
        check_cuda(cudaMallocHost(reinterpret_cast<void**>(&slot.host_raw),raw_bytes),"allocate grouped host RAW");
        check_cuda(cudaMalloc(reinterpret_cast<void**>(&slot.device_raw),raw_bytes),"allocate grouped device RAW");
        slot.raw_capacity=raw_bytes;
      }
      if(runs>slot.run_capacity) {
        if(slot.host_runs){check_cuda(cudaFreeHost(slot.host_runs),"free grouped host descriptors");slot.host_runs=nullptr;}
        if(slot.device_runs){check_cuda(cudaFree(slot.device_runs),"free grouped device descriptors");slot.device_runs=nullptr;}
        slot.run_capacity=0;
        check_cuda(cudaMallocHost(reinterpret_cast<void**>(&slot.host_runs),runs*sizeof(Evt3Run)),"allocate grouped host descriptors");
        check_cuda(cudaMalloc(reinterpret_cast<void**>(&slot.device_runs),runs*sizeof(Evt3Run)),"allocate grouped device descriptors");
        slot.run_capacity=runs;
      }
    }
  }

  void update_grouped_raw(const Evt3RunBatch& batch,const std::uint8_t* raw,
    std::size_t raw_bytes,cudaStream_t stream)
  {
    batch.validate(width_,height_,bins_,separate_,polarity_major_,bin_width_us_);
    if(!raw||raw_bytes!=batch.raw_bytes)throw std::invalid_argument("Grouped RAW buffer differs from descriptors");
    if(initialized_&&!has_grouped_epoch_)throw std::logic_error("Reset before switching grouped/event/bin update modes");
    const auto& m=batch.metadata;const bool epoch=!has_grouped_epoch_||m.epoch!=grouped_epoch_;
    if(!epoch&&(m.origin_us!=origin_timestamp_us_||m.newest_bin<newest_bin_||m.latest_us<latest_timestamp_us_))
      throw std::invalid_argument("Out-of-order grouped RAW batch");
    auto& slot=grouped_slots_[next_grouped_slot_];
    if(raw_bytes>slot.raw_capacity||batch.runs.size()>slot.run_capacity)
      throw std::length_error("Grouped staging too small; reserve_grouped_raw before replay");
    last_stream_=stream;has_stream_work_=true;
    try {
      if(slot.pending)check_cuda(cudaEventSynchronize(slot.complete),"wait for grouped RAW staging");
      std::memcpy(slot.host_raw,raw,raw_bytes);
      if(!batch.runs.empty())std::memcpy(slot.host_runs,batch.runs.data(),batch.runs.size()*sizeof(Evt3Run));
      if(epoch||m.newest_bin-newest_bin_>=static_cast<std::int64_t>(ring_bins_))
        check_cuda(cudaMemsetAsync(ring_,0,ring_bytes_,stream),"clear grouped rolling ring");
      else for(auto b=newest_bin_+1;b<=m.newest_bin;++b) {
        const auto count=pixels_*(separate_?2u:1u);
        clear_time_slot<<<static_cast<unsigned>((count+255)/256),256,0,stream>>>(ring_,pixels_,ring_bins_,b%ring_bins_,separate_,polarity_major_);
      }
      check_cuda(cudaGetLastError(),"clear grouped bins");
      check_cuda(cudaMemcpyAsync(slot.device_raw,slot.host_raw,raw_bytes,cudaMemcpyHostToDevice,stream),"upload compressed RAW");
      if(!batch.runs.empty()) {
        check_cuda(cudaMemcpyAsync(slot.device_runs,slot.host_runs,batch.runs.size()*sizeof(Evt3Run),cudaMemcpyHostToDevice,stream),"upload EVT3 run descriptors");
        constexpr unsigned threads=128;
        const auto blocks=static_cast<unsigned>((batch.runs.size()+3)/4);
        accumulate_evt3_runs<<<blocks,threads,0,stream>>>(slot.device_raw,slot.device_runs,batch.runs.size(),ring_,width_,height_,batch.input_width,
          ring_bins_,m.newest_bin-static_cast<std::int64_t>(ring_bins_-1),separate_,polarity_major_);
        check_cuda(cudaGetLastError(),"accumulate compressed EVT3 runs");
      }
      check_cuda(cudaEventRecord(slot.complete,stream),"record grouped staging completion");slot.pending=true;
      next_grouped_slot_=(next_grouped_slot_+1)%grouped_slots_.size();
      origin_timestamp_us_=m.origin_us;latest_timestamp_us_=m.latest_us;newest_bin_=m.newest_bin;
      accepted_events_=epoch?batch.accepted_events:accepted_events_+batch.accepted_events;
      grouped_epoch_=m.epoch;has_grouped_epoch_=true;initialized_=true;
    } catch(...) {
      (void)cudaStreamSynchronize(stream);initialized_=false;has_grouped_epoch_=false;throw;
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

  struct GroupedTransferSlot {
    std::uint8_t* host_raw{nullptr};std::uint16_t* device_raw{nullptr};
    Evt3Run* host_runs{nullptr};Evt3Run* device_runs{nullptr};
    std::size_t raw_capacity{0},run_capacity{0};cudaEvent_t complete{nullptr};bool pending{false};
  };
  std::vector<GroupedTransferSlot> grouped_slots_{2};std::size_t next_grouped_slot_{0};
  bool has_grouped_epoch_{false};std::uint64_t grouped_epoch_{0};
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

void EventTensorCudaBackend::update_bins_direct(
  const HistogramBinView & view, const cudaStream_t stream)
{
  impl_->update_bins_direct(view, stream);
}

void EventTensorCudaBackend::reserve_grouped_raw(std::size_t raw_bytes,std::size_t runs)
{
  impl_->reserve_grouped_raw(raw_bytes,runs);
}
void EventTensorCudaBackend::update_grouped_raw(const Evt3RunBatch& batch,
  const std::uint8_t* raw,std::size_t raw_bytes,cudaStream_t stream)
{
  impl_->update_grouped_raw(batch,raw,raw_bytes,stream);
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
