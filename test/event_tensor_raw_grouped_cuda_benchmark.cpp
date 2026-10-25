// Compare CPU histogram + direct bins with CPU run scan + compressed GPU accumulation.
// Includes RAW decode/scan, staging, H2D, GPU histogram, Tensor and completion wait.
#include "raw_evt3_benchmark_input.hpp"
#include "openeb_ros2/event_tensor_cuda_backend.hpp"
#include "openeb_ros2/evt3_runs_histogram.hpp"
#include "openeb_ros2/evt3_run_scanner.hpp"
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
  std::uint64_t bytes{0},cpu_bytes{0},over4{0},over_span{0},raw_bytes{0},descriptor_bytes{0},runs{0};
};
double mean(const std::vector<double>& v){return std::accumulate(v.begin(),v.end(),0.0)/v.size();}
double percentile(std::vector<double> v,double p){std::sort(v.begin(),v.end());return v[static_cast<std::size_t>(std::ceil(p*v.size()))-1];}
void report(const char* name,const Result& r) {
  std::cout<<std::quoted(name)<<":{\"mean_ms\":"<<mean(r.total)
    <<",\"p50_ms\":"<<percentile(r.total,.50)<<",\"p95_ms\":"<<percentile(r.total,.95)<<",\"p99_ms\":"<<percentile(r.total,.99)
    <<",\"cpu_decode_mean_ms\":"<<mean(r.decode)<<",\"cpu_export_mean_ms\":"<<mean(r.export_ms)
    <<",\"gpu_submit_mean_ms\":"<<mean(r.submit)<<",\"gpu_wait_mean_ms\":"<<mean(r.wait)
    <<",\"compressed_raw_h2d_bytes\":"<<r.raw_bytes<<",\"run_descriptor_h2d_bytes\":"<<r.descriptor_bytes<<",\"runs_transferred\":"<<r.runs
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
  // Untimed sizing pass: avoid comparing dynamic CUDA allocations with a
  // preallocated baseline. It uses the same packet boundaries and validates
  // scanner counts before either mode enters measured replay.
  std::size_t max_raw=0,max_runs=1;
  {
  openeb_ros2::Evt3RunScanner sizing(160,120,10,40000,true,true);sizing.set_geometry(raw.width,raw.height);
  for(const auto& frame:frames) {
    sizing.begin_batch(frame.end-frame.begin);std::size_t offset=0;
    raw_bench::packets(raw,frame,packet_size,[&](const std::uint8_t* p,std::size_t n){sizing.feed(p,n,offset);offset+=n;});
    max_raw=std::max(max_raw,frame.end-frame.begin);max_runs=std::max(max_runs,sizing.batch().runs.size());
    if(sizing.statistics().accepted!=frame.accepted||sizing.statistics().dropped!=frame.dropped)throw std::runtime_error("Sizing scanner differs from reference decoder");
  }
  }
  std::ofstream rows;if(!csv.empty()){rows.open(csv);if(!rows)throw std::runtime_error("Cannot write CSV");
    rows<<"frame,warmup,mode,raw_bytes,events,sensor_span_ms,event_rate_per_second,window_end_us,total_ms,cpu_decode_ms,cpu_export_ms,gpu_submit_ms,gpu_wait_ms,h2d_payload_bytes,histogram_cpu_copy_bytes,compressed_raw_h2d_bytes,run_descriptor_h2d_bytes,runs_transferred\n";rows<<std::setprecision(12);}
  int id=0,driver=0,runtime=0;cudaDeviceProp properties{};
  check(cudaGetDevice(&id));check(cudaGetDeviceProperties(&properties,id));
  check(cudaDriverGetVersion(&driver));check(cudaRuntimeGetVersion(&runtime));
  cudaStream_t stream;check(cudaStreamCreate(&stream));float* output=nullptr;
  constexpr std::size_t elements=20*160*120,bytes=elements*sizeof(float);
  check(cudaMalloc(reinterpret_cast<void**>(&output),bytes));
  std::array<Result,2> results;
  const std::array<const char*,2> names{"evt3_runs_gpu_direct","evt3_runs_grouped_gpu"};
  std::uint64_t digest=1469598103934665603ULL,direct_bounds=0;
  {
    openeb_ros2::EventTensorCudaBackend direct_gpu(160,120,10,true,true,40000,8192,{0.0},{1.0},false);
    openeb_ros2::EventTensorCudaBackend grouped_gpu(160,120,10,true,true,40000,8192,{0.0},{1.0},false);
    grouped_gpu.reserve_grouped_raw(max_raw,max_runs);
    openeb_ros2::Evt3RunsHistogram direct(160,120,10,40000,true,true);direct.set_geometry(raw.width,raw.height);
    openeb_ros2::Evt3RunScanner scanner(160,120,10,40000,true,true);scanner.set_geometry(raw.width,raw.height);scanner.reserve_runs(max_runs);
    openeb_ros2::HistogramBinView view;
    std::vector<float> reference(elements),actual(elements),cpu_reference(elements);
    for(std::size_t f=0;f<frames.size();++f) {
      const auto& frame=frames[f];const auto count=frame.accepted-(f?frames[f-1].accepted:0);
      const double span=(frame.latest-(f?frames[f-1].latest:frame.origin))/1000.0;
      for(std::size_t order=0;order<2;++order) {
        const auto mode=(order+f+order_offset)%2;
        auto& gpu=mode==0?direct_gpu:grouped_gpu;
        std::uint64_t sent=0;double decode_ms=0;
        const auto start=Clock::now();
        std::size_t offset=0;if(mode==1)scanner.begin_batch(frame.end-frame.begin);
        raw_bench::packets(raw,frame,packet_size,[&](const std::uint8_t* p,std::size_t n){
          const auto decode_start=Clock::now();if(mode==0)direct.feed(p,n);else scanner.feed(p,n,offset);offset+=n;decode_ms+=elapsed(decode_start);
        });
        const auto export_start=Clock::now();
        if(mode==0){direct.peek_dirty_bins(view);sent=view.payload_bytes();}
        else sent=scanner.batch().payload_bytes();
        const auto export_ms=elapsed(export_start);const auto submit_start=Clock::now();
        if(mode==0){gpu.update_bins_direct(view,stream);direct.acknowledge_dirty_bins();}
        else gpu.update_grouped_raw(scanner.batch(),raw.bytes.data()+frame.begin,frame.end-frame.begin,stream);
        gpu.snapshot(output,frame.window_end,stream);
        const auto submit_ms=elapsed(submit_start);const auto wait_start=Clock::now();
        check(cudaStreamSynchronize(stream));const auto wait_ms=elapsed(wait_start);const auto total=elapsed(start);
        const auto cpu_bytes=mode==0?sent:0;
        const auto raw_sent=mode==1?frame.end-frame.begin:0;
        const auto run_count=mode==1?scanner.batch().runs.size():0;
        const auto descriptors=run_count*sizeof(openeb_ros2::Evt3Run);
        if(f>=warmup) {
          auto& r=results[mode];r.total.push_back(total);r.decode.push_back(decode_ms);
          r.export_ms.push_back(export_ms);r.submit.push_back(submit_ms);r.wait.push_back(wait_ms);
          r.bytes+=sent;r.cpu_bytes+=cpu_bytes;r.raw_bytes+=raw_sent;r.descriptor_bytes+=descriptors;r.runs+=run_count;if(total>4)++r.over4;if(total>span)++r.over_span;
        }
        if(rows.is_open())rows<<f<<','<<(f<warmup)<<','<<names[mode]<<','<<frame.end-frame.begin<<','<<count<<','<<span<<','<<(span>0?count*1000.0/span:0)<<','<<frame.window_end<<','<<total<<','<<decode_ms<<','<<export_ms<<','<<submit_ms<<','<<wait_ms<<','<<sent<<','<<cpu_bytes<<','<<raw_sent<<','<<descriptors<<','<<run_count<<'\n';
        // D2H, CPU golden gather and bitwise Tensor checks are outside timing.
        check(cudaMemcpy(actual.data(),output,bytes,cudaMemcpyDeviceToHost));
        if(order==0)reference=actual;
        else if(std::memcmp(actual.data(),reference.data(),bytes))throw std::runtime_error("Paired Tensor mismatch at frame "+std::to_string(f));
      }
      direct.snapshot(cpu_reference.data(),frame.window_end);
      if(std::memcmp(actual.data(),cpu_reference.data(),bytes))throw std::runtime_error("GPU vs CPU golden Tensor mismatch");
      const auto& stats=scanner.statistics();const auto& metadata=scanner.batch().metadata;
      if(direct.statistics().accepted!=frame.accepted||stats.accepted!=frame.accepted||
         direct.statistics().dropped!=frame.dropped||stats.dropped!=frame.dropped||
         direct.origin()!=frame.origin||metadata.origin_us!=frame.origin||direct.latest()!=frame.latest||metadata.latest_us!=frame.latest||
         direct.statistics().bounds!=stats.bounds||direct.epoch()!=metadata.epoch)
        throw std::runtime_error("Grouped decoder counts or clock mismatch");
      direct_bounds=direct.statistics().bounds;
      for(float value:actual){std::uint32_t bits;std::memcpy(&bits,&value,4);digest=(digest^bits)*1099511628211ULL;}
    }
  }
  check(cudaFree(output));check(cudaStreamDestroy(stream));
  if(rows.is_open()){rows.flush();if(!rows)throw std::runtime_error("CSV write failed");}
  std::cout<<std::setprecision(12)<<"{\"scope\":\"raw_decode_to_gpu_tensor\",\"comparison\":\"compressed_gpu_histogram\",\"gpu\":"<<std::quoted(properties.name)
    <<",\"sm\":"<<properties.major*10+properties.minor<<",\"driver\":"<<driver<<",\"runtime\":"<<runtime
    <<",\"codec_decoder_sha256\":\""<<OPENEB_CODEC_DECODER_SHA256<<"\",\"raw\":"<<std::quoted(path)
    <<",\"raw_width\":"<<raw.width<<",\"raw_height\":"<<raw.height<<",\"file_payload_bytes\":"<<raw.file_payload_bytes
    <<",\"loaded_payload_bytes\":"<<raw.bytes.size()<<",\"processed_payload_bytes\":"<<frames.back().end
    <<",\"grouped_raw_capacity_bytes\":"<<max_raw<<",\"grouped_run_capacity\":"<<max_runs<<",\"grouped_staging_slots\":2,\"packet_bytes\":"<<packet_size<<",\"frames\":"<<frames.size()<<",\"warmup\":"<<warmup
    <<",\"measured_frames\":"<<frames.size()-warmup<<",\"accepted_events\":"<<frames.back().accepted
    <<",\"dropped_backward_events\":"<<frames.back().dropped<<",\"out_of_bounds_events\":"<<direct_bounds
    <<",\"sensor_duration_ms\":"<<(frames.back().latest-frames.front().origin)/1000.0
    <<",\"tensor_shape\":[1,20,120,160],\"tensor_dtype\":\"float32\",\"tensor_digest\":\""<<digest
    <<"\",\"all_outputs_identical\":true,\"cpu_reference_identical\":true,";
  report(names[0],results[0]);std::cout<<',';report(names[1],results[1]);std::cout<<"}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
