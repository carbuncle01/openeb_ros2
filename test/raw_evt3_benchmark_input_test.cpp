#include "raw_evt3_benchmark_input.hpp"
#include "openeb_ros2/evt3_runs_histogram.hpp"
#include "bin_update_mirror.hpp"
#include "openeb_ros2/evt3_run_scanner.hpp"
#include "evt3_run_mirror.hpp"
#include "openeb_ros2/evt3_grouped_stream.hpp"
#include <iostream>
#include <stdexcept>
void require(bool ok,const char* reason="RAW replay mismatch"){if(!ok)throw std::runtime_error(reason);}
void verify(const raw_bench::Raw& raw,std::size_t packet_size,std::size_t max_frames=100) {
  const auto frames=raw_bench::index(raw,packet_size,max_frames);
  raw_bench::Collector<> collector(raw.width,raw.height);
  event_camera_codecs::evt3::Decoder<raw_bench::Packet,raw_bench::Collector<>> decoder;
  decoder.setGeometry(raw.width,raw.height);
  openeb_ros2::Evt3RunsHistogram direct(160,120,10,40000,true,true),reference(160,120,10,40000,true,true);
  direct.set_geometry(raw.width,raw.height);reference.set_geometry(160,120);
  BinUpdateMirror mirror(160,120,10,4000,true,true),view_mirror(160,120,10,4000,true,true);
  openeb_ros2::HistogramBinView view;
  openeb_ros2::HistogramBinUpdates staged;
  openeb_ros2::HistogramBinUpdates updates;std::vector<float>a(direct.elements()),b(a.size()),c;
  openeb_ros2::Evt3RunScanner scanner(160,120,10,40000,true,true);scanner.set_geometry(raw.width,raw.height);
  Evt3RunMirror grouped(160,120,10,40000,true,true);
  openeb_ros2::Evt3GroupedStream live(160,120,10,40000,true,true,4000,1048576,65536);
  Evt3RunMirror live_mirror(160,120,10,40000,true,true);std::uint64_t sequence=0;
  for(const auto& frame:frames) {
    scanner.begin_batch(frame.end-frame.begin);std::size_t offset=0;
    raw_bench::packets(raw,frame,packet_size,[&](const std::uint8_t* p,std::size_t n){
      collector.events.clear();decoder.decode(p,n,&collector);direct.feed(p,n);scanner.feed(p,n,offset);offset+=n;
      const auto& live_batch=live.ingest(p,n,raw.width,raw.height,sequence++);
      if(live_batch.ready)live_mirror.apply(live_batch,p);
      for(const auto& e:collector.events)reference.event(e.timestamp_us,e.x,e.y,e.polarity);
    });
    require(collector.accepted==frame.accepted&&direct.statistics().accepted==frame.accepted,"accepted count");
    require(direct.statistics().dropped==frame.dropped,"dropped count");
    require(direct.origin()==frame.origin&&direct.latest()==frame.latest,"clock");
    direct.snapshot(a.data(),frame.window_end);reference.snapshot(b.data(),frame.window_end);
    direct.peek_dirty_bins(view);
    static_cast<openeb_ros2::HistogramBinMetadata&>(staged)=view;
    staged.values.resize(view.payload_bytes()/sizeof(float));
    for(std::size_t i=0;i<view.slots.size();++i)view.copy_slot_to(i,staged.values.data()+i*2*160*120);
    view_mirror.apply(staged);
    direct.take_dirty_bins(updates);require(staged.values==updates.values,"direct staging bytes differ");mirror.apply(updates);mirror.snapshot(c,frame.window_end);
    require(scanner.statistics().accepted==frame.accepted&&scanner.statistics().dropped==frame.dropped&&scanner.statistics().bounds==direct.statistics().bounds,"grouped scanner counts");
    grouped.apply(scanner.batch(),raw.bytes.data()+frame.begin);grouped.snapshot(c.data(),frame.window_end);
    require(a==c,"compressed run Tensor");
    require(live.statistics().accepted==frame.accepted&&live.statistics().dropped==frame.dropped,"live packet-stream counts");
    live_mirror.snapshot(c.data(),frame.window_end);require(a==c,"live packet-stream Tensor");
    require(a==b,"direct vs reference");require(a==c,"dirty mirror");view_mirror.snapshot(c,frame.window_end);require(a==c,"direct view mirror");
  }
  std::cout<<"packet_bytes="<<packet_size<<" frames="<<frames.size()<<" accepted="<<frames.back().accepted
    <<" dropped="<<frames.back().dropped<<" processed_bytes="<<frames.back().end<<'\n';
}
int main(int argc,char** argv)try {
  if(argc>1) {
    std::string raw_path;std::size_t max_bytes=16*1024*1024,max_frames=100;
    for(int i=1;i<argc;++i) {
      if(i+1>=argc)throw std::invalid_argument("Missing test option value");
      const std::string key=argv[i],value=argv[++i];
      if(key=="--raw")raw_path=value;else if(key=="--max-bytes")max_bytes=std::stoull(value);
      else if(key=="--max-frames")max_frames=std::stoull(value);else throw std::invalid_argument("Unknown test option");
    }
    if(raw_path.empty())throw std::invalid_argument("--raw is required");
    verify(raw_bench::load(raw_path,max_bytes),65536,max_frames);
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
