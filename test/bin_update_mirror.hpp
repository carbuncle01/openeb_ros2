#pragma once
#include "openeb_ros2/histogram_bin_updates.hpp"
#include <algorithm>
#include <vector>
// CPU emulation of GPU upload+gather semantics, NOT a CUDA execution test.
class BinUpdateMirror {
  std::size_t w_,h_,b_,pixels_,planes_;bool separate_,major_,ready_{false};
  std::int64_t bin_us_,origin_{0},newest_{-1};std::uint64_t epoch_{0};
  std::vector<float> ring_;
public:
  BinUpdateMirror(std::size_t w,std::size_t h,std::size_t b,std::int64_t bin_us,bool s,bool major)
    :w_(w),h_(h),b_(b),pixels_(w*h),planes_(s?2:1),separate_(s),major_(major),bin_us_(bin_us),ring_(2*b*planes_*pixels_){}
  void apply(const openeb_ros2::HistogramBinUpdates& u) {
    u.validate(w_,h_,b_,separate_,major_,bin_us_);
    if(!ready_ || epoch_!=u.epoch)std::fill(ring_.begin(),ring_.end(),0.f);
    for(std::size_t i=0;i<u.slots.size();++i)
      std::copy_n(u.values.data()+i*planes_*pixels_,planes_*pixels_,ring_.data()+u.slots[i]*planes_*pixels_);
    ready_=true;epoch_=u.epoch;origin_=u.origin_us;newest_=u.newest_bin;
  }
  void snapshot(std::vector<float>& out,std::int64_t end,const std::vector<double>& mean={0.0},const std::vector<double>& sd={1.0}) const {
    if(!ready_ || end-origin_<static_cast<std::int64_t>(b_)*bin_us_ || (end-origin_)%bin_us_)
      throw std::invalid_argument("Mirror snapshot clock");
    out.assign(b_*planes_*pixels_,0.f);
    for(std::size_t p=0;p<planes_;++p)for(std::size_t t=0;t<b_;++t) {
      const auto global=(end-origin_)/bin_us_-static_cast<std::int64_t>(b_)+static_cast<std::int64_t>(t);
      const auto c=separate_?(major_?p*b_+t:t*2+p):t;
      for(std::size_t pixel=0;pixel<pixels_;++pixel){
        float v=0;
        if(global<=newest_ && global>=newest_-static_cast<std::int64_t>(2*b_-1))
          v=ring_[(static_cast<std::size_t>(global)%(2*b_)*planes_+p)*pixels_+pixel];
        out[c*pixels_+pixel]=(v-static_cast<float>(mean[mean.size()==1?0:c]))/
          static_cast<float>(sd[sd.size()==1?0:c]);
      }
    }
  }
};
