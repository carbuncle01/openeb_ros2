#pragma once
#include "openeb_ros2/evt3_run_batch.hpp"
#include "openeb_ros2/evt3_runs_histogram.hpp"
// CPU oracle for the compressed GPU kernel; not GPU execution.
class Evt3RunMirror {
  openeb_ros2::Evt3RunsHistogram histogram_;bool ready_{false};std::uint64_t epoch_{0};
public:
  Evt3RunMirror(unsigned w,unsigned h,unsigned bins,std::int64_t window,bool separate,bool major)
    :histogram_(w,h,bins,window,separate,major){histogram_.set_geometry(w,h);}
  void apply(const openeb_ros2::Evt3RunBatch& batch,const std::uint8_t* raw) {
    const auto& m=batch.metadata;batch.validate(m.width,m.height,m.bins,m.separate,m.polarity_major,m.bin_width_us);
    if(!ready_||epoch_!=m.epoch){histogram_.reset_stream();ready_=true;epoch_=m.epoch;}
    for(const auto& run:batch.runs) {
      const auto count=run.kind==2?run.word_count:(run.kind==4?12u:8u);
      for(unsigned i=0;i<count;++i) {
        unsigned x,p;
        const auto off=2*(static_cast<std::size_t>(run.word_offset)+(run.kind==2?i:0u));
        const auto word=raw[off]|(unsigned(raw[off+1])<<8);
        if(run.kind==2){x=word&2047;p=(word>>11)&1;}
        else {if(!(word&(1u<<i)))continue;x=static_cast<std::uint16_t>(run.base_x+i);p=run.polarity;}
        if(x<batch.input_width)histogram_.event(m.origin_us+run.bin*m.bin_width_us,x*m.width/batch.input_width,run.y,p);
      }
    }
  }
  void snapshot(float* out,std::int64_t end,const std::vector<double>& mean={0.0},const std::vector<double>& sd={1.0})const {
    histogram_.snapshot(out,end,mean,sd);
  }
};
