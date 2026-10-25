#include "openeb_ros2/evt3_run_scanner.hpp"
#include "evt3_run_mirror.hpp"
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
std::vector<std::uint8_t> bytes(const std::vector<unsigned>& words){std::vector<std::uint8_t>b;for(auto w:words){b.push_back(w&255);b.push_back(w>>8);}return b;}
int main()try {
  for(bool separate:{false,true})for(bool major:{false,true})for(auto packet_size:{2u,14u,64u,65536u}) {
    openeb_ros2::Evt3RunScanner scanner(160,120,10,40000,separate,major);scanner.set_geometry(641,481);
    openeb_ros2::Evt3RunsHistogram reference(160,120,10,40000,separate,major);reference.set_geometry(641,481);
    Evt3RunMirror mirror(160,120,10,40000,separate,major);std::vector<float>a(reference.elements()),b(a.size());
    auto feed=[&](const std::vector<std::uint8_t>& data,std::size_t chunk) {
      scanner.begin_batch(data.size());
      for(std::size_t p=0;p<data.size();p+=chunk){auto n=std::min(chunk,data.size()-p);scanner.feed(data.data()+p,n,p);reference.feed(data.data()+p,n);}
      const auto& batch=scanner.batch();const auto& r=reference.statistics();const auto& s=scanner.statistics();
      require(r.accepted==s.accepted&&r.bounds==s.bounds&&r.dropped==s.dropped&&r.backward==s.backward&&r.resets==s.resets,"scanner counts differ");
      require(batch.ready==reference.ready(),"scanner readiness differs");
      if(!batch.ready)return;
      require(batch.metadata.origin_us==reference.origin()&&batch.metadata.latest_us==reference.latest()&&batch.metadata.epoch==reference.epoch(),"scanner clock/epoch differs");
      mirror.apply(batch,data.data());
      const auto end=batch.metadata.origin_us+std::max<std::int64_t>(10,batch.metadata.newest_bin)*4000;
      reference.snapshot(a.data(),end,{1.0},{2.0});mirror.snapshot(b.data(),end,{1.0},{2.0});
      require(!std::memcmp(a.data(),b.data(),a.size()*sizeof(float)),"grouped CPU tensor differs");
      auto invalid=batch;invalid.runs.push_back({0,static_cast<std::uint32_t>(data.size()),1,0,0,2,0,0});bool rejected=false;
      try{invalid.validate(160,120,10,separate,major,4000);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"invalid descriptor accepted");
    };
    feed(bytes({0x8fff}),2);feed(bytes({0x6000,0x2801}),4); // HIGH/LOW split before valid time.
    feed(bytes({0x8fff,0x6ff0,0x0002,0x2801}),8); // acquire clock in one packet.
    std::mt19937 random(42);
    for(unsigned frame=0;frame<80;++frame) {
      std::vector<unsigned> words;
      for(unsigned n=0;n<100;++n) {
        const auto t=0xfffff0ULL+frame*1000+n*8;
        words.push_back(0x8000|((t>>12)&4095));words.push_back(0x6000|(t&4095));words.push_back(random()%482);
        for(unsigned x=0;x<30;++x)words.push_back(0x2000|(random()%2048)|((random()%2)<<11));
        words.push_back(0x3800|(random()%2048));words.push_back(0x4000|(random()%4096));words.push_back(0x5000|(random()%4096));
        if((t&4095)>2){words.push_back(0x6000|((t-2)&4095));words.push_back(0x2801);}
      }
      feed(bytes(words),packet_size);
    }
    // metadata-only batch; large forward gap clears old bins.
    feed(bytes({0xe000,0xf000}),packet_size);
    feed(bytes({0x8100,0x6000,0x0001,0x2801}),packet_size); // reset after large backwards jump.
    feed(bytes({0x8101,0x6000,0x0001,0x2802}),packet_size);
    feed(bytes({0x8100,0x6000,0x0001,0x2803}),packet_size); // >4ms backwards, without HIGH rollover.
    // uint16 vector base wraps after many encoded vectors.
    std::vector<unsigned> vectors{0x0001,0x3fff};for(unsigned i=0;i<5500;++i)vectors.push_back(0x4001);
    feed(bytes(vectors),packet_size);
  }
  std::cout<<"evt3_run_scanner_test passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
