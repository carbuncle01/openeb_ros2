// Copyright 2026 openeb_ros2 contributors
// SPDX-License-Identifier: Apache-2.0
// EVT3 clock/state rules follow ros-event-camera/event_camera_codecs
// evt3_decoder.h (Copyright 2022 Bernd Pfrommer, Apache-2.0).
// Unlike the reference decoder, consecutive ADDR_X words share bin/row setup.
#ifndef OPENEB_ROS2__EVT3_RUNS_HISTOGRAM_HPP_
#define OPENEB_ROS2__EVT3_RUNS_HISTOGRAM_HPP_
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include <cstring>
#include "openeb_ros2/histogram_bin_updates.hpp"
#include "openeb_ros2/histogram_bin_view.hpp"

namespace openeb_ros2 {
// External serialization required. No ROS, CUDA, per-event storage or ownership
// of input bytes. feed() consumes the input before returning. Even-byte packets.
class Evt3RunsHistogram {
public:
  struct Statistics {
    std::uint64_t accepted{0}, bounds{0}, backward{0}, resets{0}, triggers{0}, dropped{0}, backward_max_us{0};
  };
  Evt3RunsHistogram(std::size_t width, std::size_t height, std::size_t bins,
    std::int64_t window_us, bool separate, bool polarity_major,
    std::int64_t backward_tolerance_us = 4000)
  : width_(width), height_(height), bins_(bins), separate_(separate),
    polarity_major_(polarity_major), window_us_(window_us), tolerance_(backward_tolerance_us)
  {
    if (!width || !height || !bins || bins > 1024 || window_us <= 0 ||
      window_us % static_cast<std::int64_t>(bins) || backward_tolerance_us < 0 ||
      width > 65535 || height > 65535 || width*height > SIZE_MAX / (bins*4*sizeof(float)))
    {throw std::invalid_argument("Invalid direct histogram configuration");}
    pixels_ = width*height; bin_us_ = window_us/static_cast<std::int64_t>(bins);
    ring_bins_ = bins*2; channels_ = bins*(separate ? 2:1);
    ring_.resize(ring_bins_*(separate ? 2:1)*pixels_);
    dirty_.resize(ring_bins_,true);
  }
  void set_geometry(std::uint32_t width, std::uint32_t height) {
    if (!width || !height || width > 2048 || height > 2048)
      throw std::invalid_argument("EVT3 geometry must be 1..2048");
    xs_.resize(width); ys_.resize(height);
    for (std::size_t x=0; x<width; ++x) xs_[x]=x*width_/width;
    for (std::size_t y=0; y<height; ++y) ys_[y]=y*height_/height;
    reset_stream();
  }
  void reset_stream() {
    valid_time_=false; high_=0; low_=0; y_=0; base_=0; polarity_=0;
    clear_epoch();
  }
  void clear_epoch() {
    std::fill(ring_.begin(),ring_.end(),0.f); initialized_=false;
    origin_=0; latest_=0; newest_=-1; ++epoch_; mark_all_dirty();
  }
  // Scalar reference path for event_camera_codecs callbacks and equivalence tests.
  void event(std::int64_t time_us, unsigned x, unsigned y, unsigned p) {
    if (x>=xs_.size() || y>=ys_.size()) {++stats_.bounds; return;}
    if (!prepare(time_us,1)) return;
    float* row = plane(p)+ys_[y]*width_;
    row[xs_[x]] += !separate_ && !p ? -1.f:1.f;
    ++stats_.accepted;
  }
  void feed(const std::uint8_t* bytes, std::size_t size) {
    if (size%2 || (size && !bytes)) throw std::invalid_argument("EVT3 packet must contain whole words");
    if (xs_.empty()) throw std::logic_error("EVT3 geometry is unset");
    // Match reference findValidTime: HIGH and LOW must be found in the same
    // input packet before initial acquisition; state thereafter spans packets.
    bool acquisition_high=false;
    for (std::size_t i=0; i<size; i+=2) {
      const auto word=load(bytes+i); const unsigned code=word>>12, value=word&4095;
      if (!valid_time_) {
        if (code==8) {update_high(value); acquisition_high=true;}
        else if (code==6) {low_=value; if(acquisition_high) valid_time_=true;}
        continue;
      }
      switch(code) {
        case 0: y_=value&2047; break;
        case 2: {
          std::size_t end=i+2;
          while(end<size && (bytes[end+1]>>4)==2) end+=2;
          add_run(bytes+i,(end-i)/2); i=end-2; break;
        }
        case 3: base_=value&2047; polarity_=(value>>11)&1; break;
        case 4: case 5: {
          const unsigned count=code==4 ? 12:8;
          for(unsigned b=0;b<count;++b) if(value&(1u<<b))
            event(time(),static_cast<std::uint16_t>(base_+b),y_,polarity_);
          base_=static_cast<std::uint16_t>(base_+count); break;
        }
        case 6: low_=value; break;
        case 8: update_high(value); break;
        case 10: ++stats_.triggers; break; // Tensor consumer ignores triggers, like its legacy callback.
        case 7: case 9: case 14: case 15: break;
        default: throw std::runtime_error("Unsupported EVT3 word code");
      }
    }
  }
  bool ready() const {return initialized_;}
  std::int64_t origin() const {return origin_;}
  std::int64_t latest() const {return latest_;}
  std::int64_t latest_window_end() const {
    return initialized_ && newest_+1>=static_cast<std::int64_t>(bins_) ? origin_+(newest_+1)*bin_us_:0;
  }
  std::uint64_t epoch() const {return epoch_;}
  const Statistics& statistics() const {return stats_;}
  std::size_t elements() const {return channels_*pixels_;}
  // Caller holds the state lock. Owns all exported data; CPU can resume as
  // soon as this returns. Retry after transfer failure with mark_all_dirty().
  void take_dirty_bins(HistogramBinUpdates& out) {
    if(!initialized_) throw std::logic_error("Histogram is not initialized");
    out.width=width_;out.height=height_;out.bins=bins_;
    out.separate=separate_;out.polarity_major=polarity_major_;out.epoch=epoch_;
    out.origin_us=origin_;out.latest_us=latest_;out.newest_bin=newest_;out.bin_width_us=bin_us_;
    out.slots.clear();
    for(std::size_t slot=0;slot<ring_bins_;++slot)if(dirty_[slot])out.slots.push_back(slot);
    const std::size_t planes=separate_?2:1;
    out.values.resize(out.slots.size()*planes*pixels_);
    for(std::size_t i=0;i<out.slots.size();++i)for(std::size_t p=0;p<planes;++p)
      std::memcpy(out.values.data()+(i*planes+p)*pixels_,
        ring_.data()+channel(out.slots[i],p)*pixels_,pixels_*sizeof(float));
    std::fill(dirty_.begin(),dirty_.end(),false);
  }
  // No histogram payload copy. Hold the external lock until the backend has
  // copied ALL planes into owned pinned storage and returned successfully.
  // Failed uploads leave dirty flags intact; retry with mark_all_dirty() if
  // backend state was invalidated. Do not feed/reset between peek and ack.
  void peek_dirty_bins(HistogramBinView& out) const {
    if(!initialized_)throw std::logic_error("Histogram is not initialized");
    out.width=width_;out.height=height_;out.bins=bins_;
    out.separate=separate_;out.polarity_major=polarity_major_;out.epoch=epoch_;
    out.origin_us=origin_;out.latest_us=latest_;out.newest_bin=newest_;out.bin_width_us=bin_us_;
    out.slots.clear();out.planes.clear();
    for(std::size_t slot=0;slot<ring_bins_;++slot)if(dirty_[slot]) {
      out.slots.push_back(slot);
      for(std::size_t p=0;p<(separate_?2u:1u);++p)
        out.planes.push_back(ring_.data()+channel(slot,p)*pixels_);
    }
  }
  void acknowledge_dirty_bins() {std::fill(dirty_.begin(),dirty_.end(),false);}
  void mark_all_dirty() {std::fill(dirty_.begin(),dirty_.end(),true);}
  // Same retained time range and half-open windows as EventTensorCudaBackend.
  void snapshot(float* out, std::int64_t end_us,
    const std::vector<double>& means={0.0}, const std::vector<double>& stddevs={1.0}) const {
    if (!initialized_ || end_us-origin_<window_us_ || (end_us-origin_)%bin_us_)
      throw std::invalid_argument("Snapshot is not aligned to event-bin clock");
    if (!out || (means.size()!=1 && means.size()!=channels_) ||
      (stddevs.size()!=1 && stddevs.size()!=channels_))
      throw std::invalid_argument("Invalid snapshot output/normalization shape");
    const auto end=(end_us-origin_)/bin_us_;
    const auto oldest=newest_-static_cast<std::int64_t>(ring_bins_-1);
    for(std::size_t c=0;c<channels_;++c) {
      const std::size_t p=separate_ ? (polarity_major_ ? c/bins_:c%2):0;
      const std::size_t b=separate_ ? (polarity_major_ ? c%bins_:c/2):c;
      const auto global=end-static_cast<std::int64_t>(bins_)+static_cast<std::int64_t>(b);
      const float mean=static_cast<float>(means[means.size()==1?0:c]);
      const float sd=static_cast<float>(stddevs[stddevs.size()==1?0:c]);
      if(!std::isfinite(mean)||!std::isfinite(sd)||sd<=0)throw std::invalid_argument("Invalid normalization");
      const auto slot=static_cast<std::size_t>((global%static_cast<std::int64_t>(ring_bins_)+ring_bins_)%ring_bins_);
      const float* src=ring_.data()+channel(slot,p)*pixels_;
      const bool stored=global>=oldest && global<=newest_;
      for(std::size_t pixel=0;pixel<pixels_;++pixel)
        out[c*pixels_+pixel]=((stored?src[pixel]:0.f)-mean)/sd;
    }
  }
private:
  static std::uint16_t load(const std::uint8_t* p) {return p[0]|(std::uint16_t(p[1])<<8);}
  void update_high(unsigned value) {
    const auto previous=(high_>>12)&4095;
    if(value<previous && previous-value>10) high_+=1ULL<<24;
    high_=(high_&~((1ULL<<24)-1))|(std::uint64_t(value)<<12);
  }
  std::int64_t time() const {return static_cast<std::int64_t>(high_|low_);}
  std::size_t channel(std::size_t slot,std::size_t p) const {
    return separate_ ? (polarity_major_?p*ring_bins_+slot:slot*2+p):slot;
  }
  float* plane(unsigned p) {return ring_.data()+channel(newest_%ring_bins_,p?0:1)*pixels_;}
  bool prepare(std::int64_t time_us, std::size_t count) {
    if(initialized_ && time_us<latest_) {
      stats_.backward+=count;
      stats_.backward_max_us=std::max(stats_.backward_max_us,static_cast<std::uint64_t>(latest_-time_us));
      if(latest_-time_us<=tolerance_) {stats_.dropped+=count; return false;}
      ++stats_.resets; clear_epoch();
    }
    if(!initialized_) {origin_=time_us; initialized_=true;}
    const auto bin=(time_us-origin_)/bin_us_;
    if(bin>newest_) {
      if(bin-newest_>=static_cast<std::int64_t>(ring_bins_)) {
        std::fill(ring_.begin(),ring_.end(),0.f);mark_all_dirty();
      } else for(auto b=newest_+1;b<=bin;++b) {
        dirty_[b%ring_bins_]=true;
        for(std::size_t p=0;p<(separate_?2u:1u);++p)
          std::fill_n(ring_.data()+channel(b%ring_bins_,p)*pixels_,pixels_,0.f);
      }
      newest_=bin;
    }
    latest_=time_us; dirty_[bin%ring_bins_]=true; return true;
  }
  void add_run(const std::uint8_t* bytes,std::size_t count) {
    if(y_>=ys_.size()) {stats_.bounds+=count;return;}
    bool prepared=false,accept=false;float *on=nullptr,*off=nullptr;
    for(std::size_t i=0;i<count;++i) {
      const unsigned word=load(bytes+2*i),x=word&2047;
      if(x>=xs_.size()) {++stats_.bounds; continue;}
      if(!prepared) {
        accept=prepare(time(),1); prepared=true;
        if(accept){on=plane(1)+ys_[y_]*width_;off=plane(0)+ys_[y_]*width_;}
      } else if(!accept) {++stats_.backward; ++stats_.dropped;}
      if(accept){const bool positive=word&2048; (positive?on:off)[xs_[x]]+=!separate_&&!positive?-1.f:1.f; ++stats_.accepted;}
    }
  }
  std::size_t width_,height_,bins_,pixels_{0},ring_bins_{0},channels_{0};
  bool separate_,polarity_major_,initialized_{false},valid_time_{false};
  std::int64_t window_us_,tolerance_,bin_us_{0},origin_{0},latest_{0},newest_{-1};
  std::uint64_t epoch_{0},high_{0};
  unsigned low_{0},y_{0};std::uint16_t base_{0};unsigned polarity_{0};
  std::vector<bool> dirty_;
  std::vector<float> ring_;std::vector<std::size_t> xs_,ys_;Statistics stats_;
};
} // namespace openeb_ros2
#endif
