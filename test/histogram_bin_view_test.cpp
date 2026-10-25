#include "openeb_ros2/evt3_runs_histogram.hpp"
#include "bin_update_mirror.hpp"
#include <iostream>
#include <stdexcept>
#include <cstring>
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
openeb_ros2::HistogramBinUpdates stage(const openeb_ros2::HistogramBinView& view) {
  view.validate(view.width,view.height,view.bins,view.separate,view.polarity_major,view.bin_width_us);
  openeb_ros2::HistogramBinUpdates out;static_cast<openeb_ros2::HistogramBinMetadata&>(out)=view;
  out.values.resize(view.payload_bytes()/sizeof(float));
  for(std::size_t i=0;i<view.slots.size();++i)view.copy_slot_to(i,out.values.data()+i*view.width*view.height*(view.separate?2u:1u));
  return out;
}
int main()try {
  for(bool separate:{false,true})for(bool major:{false,true}) {
    openeb_ros2::Evt3RunsHistogram h(4,3,2,2000,separate,major);h.set_geometry(8,6);
    openeb_ros2::HistogramBinView view,retry;openeb_ros2::HistogramBinUpdates legacy;
    BinUpdateMirror mirror(4,3,2,1000,separate,major);
    std::vector<float> expected(h.elements()),actual;
    auto compare=[&](std::int64_t end){
      h.peek_dirty_bins(view);auto pinned=stage(view);
      h.peek_dirty_bins(retry);require(view.slots==retry.slots,"peek consumed dirty flags");
      h.take_dirty_bins(legacy);
      require(pinned.slots==legacy.slots&&pinned.values==legacy.values,"direct staging differs from owned export");
      mirror.apply(pinned);h.snapshot(expected.data(),end,{1.0},{2.0});mirror.snapshot(actual,end,{1.0},{2.0});
      require(expected==actual,"20ch-equivalent gather differs");
      h.acknowledge_dirty_bins();h.peek_dirty_bins(retry);require(retry.slots.empty(),"ack did not clear flags");
    };
    h.event(1000,2,2,1);compare(3000);compare(3000);
    for(unsigned i=0;i<40;++i){h.event(1100+i,3,2,i%2);compare(3000);}
    for(auto t:{2000,3000,4000,5000,6000,18000,20000}){h.event(t,3,3,t%2);compare(t+2000);compare(t+20000);}
    h.reset_stream();h.event(10,1,1,1);compare(2010);
    h.event(11,1,1,0);h.peek_dirty_bins(view);auto owned=stage(view);
    const auto before=owned.values;h.event(12,1,1,1);require(owned.values==before,"staging aliases histogram");
    h.mark_all_dirty();compare(2010);
    h.mark_all_dirty();h.peek_dirty_bins(view);view.planes[0]=nullptr;
    bool rejected=false;try{stage(view);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"null source accepted");
    h.peek_dirty_bins(view);view.planes.pop_back();rejected=false;
    try{stage(view);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"plane count mismatch accepted");
    h.peek_dirty_bins(view);view.slots[1]=view.slots[0];rejected=false;
    try{stage(view);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"duplicate slot accepted");
  }
  std::cout<<"histogram_bin_view_test passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
