#include "raw_evt3_benchmark_input.hpp"
#include "openeb_ros2/evt3_runs_histogram.hpp"
#include "bin_update_mirror.hpp"
#include <iostream>
#include <stdexcept>
void require(bool ok,const char* reason="RAW replay mismatch"){if(!ok)throw std::runtime_error(reason);}
void verify(const raw_bench::Raw& raw,std::size_t packet_size) {
  const auto frames=raw_bench::index(raw,packet_size,100);
  raw_bench::Collector<> collector(raw.width,raw.height);
  event_camera_codecs::evt3::Decoder<raw_bench::Packet,raw_bench::Collector<>> decoder;
  decoder.setGeometry(raw.width,raw.height);
  openeb_ros2::Evt3RunsHistogram direct(160,120,10,40000,true,true),reference(160,120,10,40000,true,true);
  direct.set_geometry(raw.width,raw.height);reference.set_geometry(160,120);
  BinUpdateMirror mirror(160,120,10,4000,true,true);
  openeb_ros2::HistogramBinUpdates updates;std::vector<float>a(direct.elements()),b(a.size()),c;
  for(const auto& frame:frames) {
    raw_bench::packets(raw,frame,packet_size,[&](const std::uint8_t* p,std::size_t n){
      collector.events.clear();decoder.decode(p,n,&collector);direct.feed(p,n);
      for(const auto& e:collector.events)reference.event(e.timestamp_us,e.x,e.y,e.polarity);
    });
    require(collector.accepted==frame.accepted&&direct.statistics().accepted==frame.accepted,"accepted count");
    require(direct.statistics().dropped==frame.dropped,"dropped count");
    require(direct.origin()==frame.origin&&direct.latest()==frame.latest,"clock");
    direct.snapshot(a.data(),frame.window_end);reference.snapshot(b.data(),frame.window_end);
    direct.take_dirty_bins(updates);mirror.apply(updates);mirror.snapshot(c,frame.window_end);
    require(a==b,"direct vs reference");require(a==c,"dirty mirror");
  }
  std::cout<<"packet_bytes="<<packet_size<<" frames="<<frames.size()<<" accepted="<<frames.back().accepted
    <<" dropped="<<frames.back().dropped<<" processed_bytes="<<frames.back().end<<'\n';
}
int main(int argc,char** argv)try {
  if(argc==3&&std::string(argv[1])=="--raw") {
    verify(raw_bench::load(argv[2],16*1024*1024),65536);
  } else {
    raw_bench::Raw raw;raw.width=641;raw.height=481;
    auto word=[&](unsigned w){raw.bytes.push_back(w&255);raw.bytes.push_back(w>>8);};
    for(unsigned t=0;t<220000;t+=100) {
      const auto us=1000+t;word(0x8000|((us>>12)&4095));word(0x6000|(us&4095));
      word(t%482);word(0x2800|(t%642));word(0x2000|((t+7)%642));
      word(0x3802);word(0x4007);word(0x5003);
      if(t%1000==0&&(us&4095)>0){word(0x6000|((us-1)&4095));word(0x2803);} // tolerated drop
    }
    for(auto packet:{14u,64u,65536u})verify(raw,packet);
    raw_bench::Collector<> c(640,480,false);c.eventCD(10000000,1,1,1);
    c.eventCD(6000000,1,1,1);require(c.dropped==1,"4ms reversal must drop");
    bool large=false;try{c.eventCD(5999000,1,1,1);}catch(const std::runtime_error&){large=true;}
    require(large,"Large reversal must fail");
    bool rejected=false;try{raw_bench::index(raw,3,0);}catch(const std::invalid_argument&){rejected=true;}require(rejected);
  }
  std::cout<<"raw_evt3_benchmark_input_test passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
