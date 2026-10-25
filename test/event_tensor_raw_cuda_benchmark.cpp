// Actual production CUDA backend and two EVT3 decode paths, without ROS/USB.
#include "raw_evt3_benchmark_input.hpp"
#include "openeb_ros2/event_tensor_cuda_backend.hpp"
#include "openeb_ros2/evt3_runs_histogram.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric>
using Clock=std::chrono::steady_clock;
void check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
double elapsed(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
struct Result {
  std::vector<double> total,decode;std::uint64_t bytes{0},over4{0},over_span{0};
};
double percentile(std::vector<double> v,double p){std::sort(v.begin(),v.end());return v[static_cast<std::size_t>(std::ceil(p*v.size()))-1];}
void report(const char* name,const Result& r) {
  std::cout<<std::quoted(name)<<":{\"mean_ms\":"<<std::accumulate(r.total.begin(),r.total.end(),0.0)/r.total.size()
    <<",\"p50_ms\":"<<percentile(r.total,.50)<<",\"p95_ms\":"<<percentile(r.total,.95)<<",\"p99_ms\":"<<percentile(r.total,.99)
    <<",\"cpu_decode_mean_ms\":"<<std::accumulate(r.decode.begin(),r.decode.end(),0.0)/r.decode.size()
    <<",\"h2d_payload_bytes\":"<<r.bytes<<",\"over_4ms\":"<<r.over4<<",\"over_sensor_span\":"<<r.over_span<<"}";
}
int main(int argc,char**argv)try {
  std::string path,csv;std::size_t packet_size=65536,max_frames=1000,warmup=30,order_offset=0;
  std::uint64_t max_bytes=268435456;
  for(int i=1;i<argc;++i) {
    if(i+1>=argc)throw std::invalid_argument("Missing option value");
    const std::string key=argv[i],value=argv[++i];
    if(key!="--raw"&&key!="--csv"&&(value.empty()||value.find_first_not_of("0123456789")!=std::string::npos))
      throw std::invalid_argument("Require a nonnegative integer for "+key);
    if(key=="--raw")path=value;else if(key=="--csv")csv=value;
    else if(key=="--packet-bytes")packet_size=std::stoull(value);
    else if(key=="--max-frames")max_frames=std::stoull(value);
    else if(key=="--max-bytes")max_bytes=std::stoull(value);
    else if(key=="--warmup")warmup=std::stoull(value);
    else if(key=="--order-offset")order_offset=std::stoull(value)%3;
    else throw std::invalid_argument("Unknown option: "+key);
  }
  if(path.empty()||warmup<1||warmup>1000000000)throw std::invalid_argument("--raw FILE and warmup>=1 required");
  const auto raw=raw_bench::load(path,max_bytes);const auto frames=raw_bench::index(raw,packet_size,max_frames);
  if(frames.size()<warmup+10)throw std::runtime_error("Need >=warmup+10 frames; increase max-bytes/max-frames or shorten warmup");
  std::ofstream rows;if(!csv.empty()){rows.open(csv);if(!rows)throw std::runtime_error("Cannot write CSV");
    rows<<"frame,warmup,mode,raw_bytes,events,sensor_span_ms,event_rate_per_second,window_end_us,total_ms,cpu_decode_ms,h2d_payload_bytes\n";rows<<std::setprecision(12);}
  int id=0,driver=0,runtime=0;cudaDeviceProp properties{};
  check(cudaGetDevice(&id));check(cudaGetDeviceProperties(&properties,id));
  check(cudaDriverGetVersion(&driver));check(cudaRuntimeGetVersion(&runtime));
  cudaStream_t stream;check(cudaStreamCreate(&stream));float* output=nullptr;
  constexpr std::size_t elements=20*160*120,bytes=elements*sizeof(float);
  check(cudaMalloc(reinterpret_cast<void**>(&output),bytes));
  std::array<Result,3> results;const std::array<const char*,3> names{"events","evt3_runs","evt3_runs_gpu"};
  std::uint64_t digest=1469598103934665603ULL,direct_bounds=0;
  {
    raw_bench::Collector<openeb_ros2::CudaEvent> collector(raw.width,raw.height);
    event_camera_codecs::evt3::Decoder<raw_bench::Packet,decltype(collector)> decoder;decoder.setGeometry(raw.width,raw.height);
    collector.events.reserve(16384); // Same initial reserve as the default async node.
    openeb_ros2::EventTensorCudaBackend events_gpu(160,120,10,true,true,40000,8192,{0.0},{1.0});
    openeb_ros2::EventTensorCudaBackend bins_gpu(160,120,10,true,true,40000,8192,{0.0},{1.0},false);
    openeb_ros2::Evt3RunsHistogram full(160,120,10,40000,true,true),dirty(160,120,10,40000,true,true);
    full.set_geometry(raw.width,raw.height);dirty.set_geometry(raw.width,raw.height);
    openeb_ros2::HistogramBinUpdates updates;
    std::vector<float> host(elements),reference(elements),actual(elements);
    for(std::size_t f=0;f<frames.size();++f) {
      const auto& frame=frames[f];const auto previous=f?frames[f-1].accepted:0;
      const auto count=frame.accepted-previous;
      const double span=(frame.latest-(f?frames[f-1].latest:frame.origin))/1000.0;
      for(std::size_t order=0;order<3;++order) {
        const auto mode=(order+f+order_offset)%3;
        std::uint64_t sent=0;double decode_ms=0;
        const auto start=Clock::now();
        raw_bench::packets(raw,frame,packet_size,[&](const std::uint8_t* p,std::size_t n){
          const auto decode_start=Clock::now();
          if(mode==0){collector.events.clear();decoder.decode(p,n,&collector);}
          else if(mode==1)full.feed(p,n);else dirty.feed(p,n);
          decode_ms+=elapsed(decode_start);
          if(mode==0){
            for(std::size_t offset=0;offset<collector.events.size();offset+=8192)
              events_gpu.update(collector.events.data()+offset,std::min<std::size_t>(8192,collector.events.size()-offset),stream);
            sent+=collector.events.size()*sizeof(openeb_ros2::CudaEvent);
          }
        });
        if(mode==0)events_gpu.snapshot(output,frame.window_end,stream);
        else if(mode==1){full.snapshot(host.data(),frame.window_end);check(cudaMemcpyAsync(output,host.data(),bytes,cudaMemcpyHostToDevice,stream));sent=bytes;}
        else {dirty.take_dirty_bins(updates);sent=updates.values.size()*sizeof(float);bins_gpu.update_bins(updates,stream);bins_gpu.snapshot(output,frame.window_end,stream);}
        check(cudaStreamSynchronize(stream));const auto total=elapsed(start);
        if(f>=warmup) {
          auto& r=results[mode];r.total.push_back(total);r.decode.push_back(decode_ms);r.bytes+=sent;
          if(total>4)++r.over4;if(total>span)++r.over_span;
        }
        if(rows.is_open())rows<<f<<','<<(f<warmup)<<','<<names[mode]<<','<<frame.end-frame.begin<<','<<count<<','<<span<<','<<(span>0?count*1000.0/span:0)<<','<<frame.window_end<<','<<total<<','<<decode_ms<<','<<sent<<'\n';
        // D2H and equivalence checks are explicitly outside all timed sections.
        check(cudaMemcpy(actual.data(),output,bytes,cudaMemcpyDeviceToHost));
        if(order==0)reference=actual;else if(actual!=reference)
          throw std::runtime_error("Tensor mismatch at frame "+std::to_string(f)+", mode "+names[mode]);
      }
      if(collector.accepted!=frame.accepted||full.statistics().accepted!=frame.accepted||dirty.statistics().accepted!=frame.accepted||
        collector.dropped!=frame.dropped||full.statistics().dropped!=frame.dropped||dirty.statistics().dropped!=frame.dropped||
        collector.bounds!=frame.bounds||full.statistics().bounds!=dirty.statistics().bounds||
        full.origin()!=frame.origin||dirty.origin()!=frame.origin||full.latest()!=frame.latest||dirty.latest()!=frame.latest)
        throw std::runtime_error("Decode counts or clock mismatch");
      direct_bounds=full.statistics().bounds;
      for(float value:actual){std::uint32_t bits;std::memcpy(&bits,&value,4);digest=(digest^bits)*1099511628211ULL;}
    }
  }
  check(cudaFree(output));check(cudaStreamDestroy(stream));
  if(rows.is_open()){rows.flush();if(!rows)throw std::runtime_error("CSV write failed");}
  std::cout<<std::setprecision(12)<<"{\"scope\":\"raw_decode_to_gpu_tensor\",\"gpu\":"<<std::quoted(properties.name)
    <<",\"sm\":"<<properties.major*10+properties.minor<<",\"driver\":"<<driver<<",\"runtime\":"<<runtime
    <<",\"codec_decoder_sha256\":\""<<OPENEB_CODEC_DECODER_SHA256<<"\",\"raw\":"<<std::quoted(path)
    <<",\"raw_width\":"<<raw.width<<",\"raw_height\":"<<raw.height<<",\"file_payload_bytes\":"<<raw.file_payload_bytes
    <<",\"loaded_payload_bytes\":"<<raw.bytes.size()<<",\"processed_payload_bytes\":"<<frames.back().end
    <<",\"gpu_chunk_events\":8192,\"decode_initial_reserve\":16384,\"packet_bytes\":"<<packet_size<<",\"frames\":"<<frames.size()<<",\"warmup\":"<<warmup
    <<",\"measured_frames\":"<<frames.size()-warmup<<",\"accepted_events\":"<<frames.back().accepted
    <<",\"dropped_backward_events\":"<<frames.back().dropped<<",\"out_of_bounds_events\":"<<direct_bounds
    <<",\"sensor_duration_ms\":"<<(frames.back().latest-frames.front().origin)/1000.0
    <<",\"tensor_digest\":\""<<digest<<"\",\"all_outputs_identical\":true,";
  for(unsigned mode=0;mode<3;++mode){if(mode)std::cout<<',';report(names[mode],results[mode]);}std::cout<<"}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
