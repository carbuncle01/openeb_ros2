// Paired ablation: identical RAW decode, H2D payload and GPU gather. Only the
// CPU histogram export/staging path differs. No ROS, USB or DNN inference.
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
  std::vector<double> total,decode,export_ms,submit,wait;
  std::uint64_t bytes{0},cpu_bytes{0},over4{0},over_span{0};
};
double mean(const std::vector<double>& v){return std::accumulate(v.begin(),v.end(),0.0)/v.size();}
double percentile(std::vector<double> v,double p){std::sort(v.begin(),v.end());return v[static_cast<std::size_t>(std::ceil(p*v.size()))-1];}
void report(const char* name,const Result& r) {
  std::cout<<std::quoted(name)<<":{\"mean_ms\":"<<mean(r.total)
    <<",\"p50_ms\":"<<percentile(r.total,.50)<<",\"p95_ms\":"<<percentile(r.total,.95)<<",\"p99_ms\":"<<percentile(r.total,.99)
    <<",\"cpu_decode_mean_ms\":"<<mean(r.decode)<<",\"cpu_export_mean_ms\":"<<mean(r.export_ms)
    <<",\"gpu_submit_mean_ms\":"<<mean(r.submit)<<",\"gpu_wait_mean_ms\":"<<mean(r.wait)
    <<",\"h2d_payload_bytes\":"<<r.bytes<<",\"histogram_cpu_copy_bytes\":"<<r.cpu_bytes
    <<",\"over_4ms\":"<<r.over4<<",\"over_sensor_span\":"<<r.over_span<<"}";
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
    else if(key=="--order-offset")order_offset=std::stoull(value)%2;
    else throw std::invalid_argument("Unknown option: "+key);
  }
  if(path.empty()||warmup<1||warmup>1000000000)throw std::invalid_argument("--raw FILE and warmup>=1 required");
  const auto raw=raw_bench::load(path,max_bytes);const auto frames=raw_bench::index(raw,packet_size,max_frames);
  if(frames.size()<warmup+10)throw std::runtime_error("Need >=warmup+10 frames; increase max-bytes/max-frames or shorten warmup");
  std::ofstream rows;if(!csv.empty()){rows.open(csv);if(!rows)throw std::runtime_error("Cannot write CSV");
    rows<<"frame,warmup,mode,raw_bytes,events,sensor_span_ms,event_rate_per_second,window_end_us,total_ms,cpu_decode_ms,cpu_export_ms,gpu_submit_ms,gpu_wait_ms,h2d_payload_bytes,histogram_cpu_copy_bytes\n";rows<<std::setprecision(12);}
  int id=0,driver=0,runtime=0;cudaDeviceProp properties{};
  check(cudaGetDevice(&id));check(cudaGetDeviceProperties(&properties,id));
  check(cudaDriverGetVersion(&driver));check(cudaRuntimeGetVersion(&runtime));
  cudaStream_t stream;check(cudaStreamCreate(&stream));float* output=nullptr;
  constexpr std::size_t elements=20*160*120,bytes=elements*sizeof(float);
  check(cudaMalloc(reinterpret_cast<void**>(&output),bytes));
  std::array<Result,2> results;
  const std::array<const char*,2> names{"evt3_runs_gpu","evt3_runs_gpu_direct"};
  std::uint64_t digest=1469598103934665603ULL,direct_bounds=0;
  {
    openeb_ros2::EventTensorCudaBackend legacy_gpu(160,120,10,true,true,40000,8192,{0.0},{1.0},false);
    openeb_ros2::EventTensorCudaBackend direct_gpu(160,120,10,true,true,40000,8192,{0.0},{1.0},false);
    openeb_ros2::Evt3RunsHistogram legacy(160,120,10,40000,true,true),direct(160,120,10,40000,true,true);
    legacy.set_geometry(raw.width,raw.height);direct.set_geometry(raw.width,raw.height);
    std::array<openeb_ros2::Evt3RunsHistogram*,2> histograms{&legacy,&direct};
    std::array<openeb_ros2::EventTensorCudaBackend*,2> backends{&legacy_gpu,&direct_gpu};
    openeb_ros2::HistogramBinUpdates updates;openeb_ros2::HistogramBinView view;
    std::vector<float> reference(elements),actual(elements),cpu_reference(elements);
    for(std::size_t f=0;f<frames.size();++f) {
      const auto& frame=frames[f];const auto count=frame.accepted-(f?frames[f-1].accepted:0);
      const double span=(frame.latest-(f?frames[f-1].latest:frame.origin))/1000.0;
      std::array<std::uint64_t,2> payloads{};
      for(std::size_t order=0;order<2;++order) {
        const auto mode=(order+f+order_offset)%2;
        auto& h=*histograms[mode];auto& gpu=*backends[mode];
        std::uint64_t sent=0;double decode_ms=0;
        const auto start=Clock::now();
        raw_bench::packets(raw,frame,packet_size,[&](const std::uint8_t* p,std::size_t n){
          const auto decode_start=Clock::now();h.feed(p,n);decode_ms+=elapsed(decode_start);
        });
        const auto export_start=Clock::now();
        if(mode==0){h.take_dirty_bins(updates);sent=updates.values.size()*sizeof(float);}
        else {h.peek_dirty_bins(view);sent=view.payload_bytes();}
        const auto export_ms=elapsed(export_start);const auto submit_start=Clock::now();
        if(mode==0)gpu.update_bins(updates,stream);
        else {gpu.update_bins_direct(view,stream);h.acknowledge_dirty_bins();}
        gpu.snapshot(output,frame.window_end,stream);
        const auto submit_ms=elapsed(submit_start);const auto wait_start=Clock::now();
        check(cudaStreamSynchronize(stream));const auto wait_ms=elapsed(wait_start);const auto total=elapsed(start);
        const auto cpu_bytes=sent*(mode==0?2u:1u);payloads[mode]=sent;
        if(f>=warmup) {
          auto& r=results[mode];r.total.push_back(total);r.decode.push_back(decode_ms);
          r.export_ms.push_back(export_ms);r.submit.push_back(submit_ms);r.wait.push_back(wait_ms);
          r.bytes+=sent;r.cpu_bytes+=cpu_bytes;if(total>4)++r.over4;if(total>span)++r.over_span;
        }
        if(rows.is_open())rows<<f<<','<<(f<warmup)<<','<<names[mode]<<','<<frame.end-frame.begin<<','<<count<<','<<span<<','<<(span>0?count*1000.0/span:0)<<','<<frame.window_end<<','<<total<<','<<decode_ms<<','<<export_ms<<','<<submit_ms<<','<<wait_ms<<','<<sent<<','<<cpu_bytes<<'\n';
        // D2H, CPU golden gather and bitwise Tensor checks are outside timing.
        check(cudaMemcpy(actual.data(),output,bytes,cudaMemcpyDeviceToHost));
        h.snapshot(cpu_reference.data(),frame.window_end);
        if(std::memcmp(actual.data(),cpu_reference.data(),bytes))throw std::runtime_error("GPU vs CPU Tensor mismatch: "+std::string(names[mode]));
        if(order==0)reference=actual;
        else if(std::memcmp(actual.data(),reference.data(),bytes))throw std::runtime_error("Paired Tensor mismatch at frame "+std::to_string(f));
      }
      if(payloads[0]!=payloads[1])throw std::runtime_error("H2D payload differs between copy paths");
      for(auto* h:histograms)if(h->statistics().accepted!=frame.accepted||h->statistics().dropped!=frame.dropped||h->origin()!=frame.origin||h->latest()!=frame.latest)
        throw std::runtime_error("Decode counts or clock mismatch");
      if(legacy.statistics().bounds!=direct.statistics().bounds)throw std::runtime_error("Bounds count mismatch");
      direct_bounds=direct.statistics().bounds;
      for(float value:actual){std::uint32_t bits;std::memcpy(&bits,&value,4);digest=(digest^bits)*1099511628211ULL;}
    }
  }
  check(cudaFree(output));check(cudaStreamDestroy(stream));
  if(rows.is_open()){rows.flush();if(!rows)throw std::runtime_error("CSV write failed");}
  std::cout<<std::setprecision(12)<<"{\"scope\":\"raw_decode_to_gpu_tensor\",\"comparison\":\"histogram_cpu_copy\",\"gpu\":"<<std::quoted(properties.name)
    <<",\"sm\":"<<properties.major*10+properties.minor<<",\"driver\":"<<driver<<",\"runtime\":"<<runtime
    <<",\"codec_decoder_sha256\":\""<<OPENEB_CODEC_DECODER_SHA256<<"\",\"raw\":"<<std::quoted(path)
    <<",\"raw_width\":"<<raw.width<<",\"raw_height\":"<<raw.height<<",\"file_payload_bytes\":"<<raw.file_payload_bytes
    <<",\"loaded_payload_bytes\":"<<raw.bytes.size()<<",\"processed_payload_bytes\":"<<frames.back().end
    <<",\"packet_bytes\":"<<packet_size<<",\"frames\":"<<frames.size()<<",\"warmup\":"<<warmup
    <<",\"measured_frames\":"<<frames.size()-warmup<<",\"accepted_events\":"<<frames.back().accepted
    <<",\"dropped_backward_events\":"<<frames.back().dropped<<",\"out_of_bounds_events\":"<<direct_bounds
    <<",\"sensor_duration_ms\":"<<(frames.back().latest-frames.front().origin)/1000.0
    <<",\"tensor_shape\":[1,20,120,160],\"tensor_dtype\":\"float32\",\"tensor_digest\":\""<<digest
    <<"\",\"all_outputs_identical\":true,\"cpu_reference_identical\":true,";
  report(names[0],results[0]);std::cout<<',';report(names[1],results[1]);std::cout<<"}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
