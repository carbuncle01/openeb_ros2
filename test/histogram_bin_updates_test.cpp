#include "openeb_ros2/evt3_runs_histogram.hpp"
#include "bin_update_mirror.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main()try {
  for(bool separate:{false,true})for(bool major:{false,true}) {
    openeb_ros2::Evt3RunsHistogram h(4,3,2,2000,separate,major);h.set_geometry(8,6);
    BinUpdateMirror mirror(4,3,2,1000,separate,major);openeb_ros2::HistogramBinUpdates updates;
    std::vector<float> a(h.elements()),b;
    const auto compare=[&](std::int64_t end){h.snapshot(a.data(),end,{1.0},{2.0});mirror.snapshot(b,end,{1.0},{2.0});require(a==b,"dirty-bin reconstruction differs");};
    h.event(1000,2,2,1);h.take_dirty_bins(updates);require(updates.slots.size()==4,"initial full ring missing");mirror.apply(updates);compare(3000);
    h.take_dirty_bins(updates);require(updates.values.empty(),"unchanged bins were resent");mirror.apply(updates);compare(3000);
    h.event(1200,3,2,0);h.take_dirty_bins(updates);require(updates.slots.size()==1,"partial bin update not compact");
    const auto owned=updates.values;h.event(1300,2,2,1);require(owned==updates.values,"export aliases live CPU state");
    mirror.apply(updates); // It is an older version; next absolute update must fix it, not add it twice.
    h.take_dirty_bins(updates);mirror.apply(updates);compare(3000);mirror.apply(updates);compare(3000);
    for(auto t:{2000,3000,4000,5000,6000,18000,20000}) {
      h.event(t,2,3,t%3==0);h.take_dirty_bins(updates);mirror.apply(updates);
      compare(t+2000);compare(t+20000);
    }
    const auto epoch=h.epoch();h.reset_stream();h.event(10,1,1,0);
    h.take_dirty_bins(updates);require(updates.epoch!=epoch,"epoch did not change");mirror.apply(updates);compare(2010);
    h.event(11,2,1,1);h.take_dirty_bins(updates); // Simulate a failed upload followed by CPU progress.
    h.event(12,3,1,1);h.mark_all_dirty();h.take_dirty_bins(updates);require(updates.slots.size()==4,"retry not complete");mirror.apply(updates);compare(2010);
    h.event(11,2,1,1);require(h.statistics().dropped==1,"small backward timestamp not tolerated");h.take_dirty_bins(updates);require(updates.slots.empty(),"dropped event marked dirty");
    auto invalid=updates;invalid.slots={0,0};invalid.values.resize(2*(separate?2:1)*12);
    bool rejected=false;try{invalid.validate(4,3,2,separate,major,1000);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"duplicate slots accepted");
  }
  std::cout<<"histogram_bin_updates_test passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
