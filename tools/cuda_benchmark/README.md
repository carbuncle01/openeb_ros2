# NVIDIAノートPCでの単独CUDA比較

Jetsonは必須ではない。CUDA対応NVIDIA GPU、対応driver/CUDA Toolkit、C++17 compiler、
CMake 3.24以上、sweep用Python 3を使う。LinuxまたはWindowsのWSL2を想定。
ROS、OpenEB SDK、カメラ、TensorRT、Jetsonライブラリは不要。
このdirectoryのCMakeはpackage本体のament entry pointと独立しており、
本番と同じsrc/event_tensor_cuda_backend.cuを直接ビルドする。
AMD/Intel GPU・Apple GPUではこのCUDAコードをそのまま実行できない。

- [NVIDIA CUDA on WSL](https://docs.nvidia.com/cuda/wsl-user-guide/)
- [CMake CUDA_ARCHITECTURES](https://cmake.org/cmake/help/latest/prop_tgt/CUDA_ARCHITECTURES.html)

## 実行

openeb_ros2のcheckout rootで実行する。kart全体のビルドやROS環境のsourceは不要。
WSL2のGPU設定は上の公式手順に従い、CUDA Toolkitと対応host compilerを用意する。

```bash
nvidia-smi
nvcc --version
cmake --version
cmake -S tools/cuda_benchmark -B build/cuda_benchmark -DCMAKE_BUILD_TYPE=Release
cmake --build build/cuda_benchmark -j 2
ctest --test-dir build/cuda_benchmark --output-on-failure
python3 tools/cuda_benchmark/sweep.py \
  --binary build/cuda_benchmark/event_tensor_cuda_load_benchmark \
  --output results/cuda_sweep_01 --iterations 300 --repeats 3
```

resultsは未作成directoryを指定する。GPU名・SM・driver/runtime version、source hash、
全runのJSON、stdout/stderr、summary.csvを保存する。
status=completeと全runのall_outputs_identical=trueを確認する。
途中失敗はstatus=failed、実行中断はrunningで残り、完了と区別する。
CMakeは既定で接続GPUのarchitectureをnativeで選ぶ。
別GPU向けやGPU非表示のbuildでは、適切なSMを明示する:
`-DCMAKE_CUDA_ARCHITECTURES=<対象GPUのSM番号>`。
Windows nativeのmulti-config buildは別の実行ファイル位置になるため、この手順はWSL2を基準とする。

## 比較内容

### 1. 転送・Tensor構成のみ

```bash
./build/cuda_benchmark/histogram_bin_cuda_benchmark --iterations 1000 --events 100000
```

共通のCPU histogramを先に作り、完成Tensor転送と変更bin転送＋GPU構成を比較する。
CPU decode/集計を計測に含めないので、この結果だけで高イベント時の全体性能を判断しない。

### 2. 集計も含むイベント量別比較

```bash
./build/cuda_benchmark/event_tensor_cuda_load_benchmark \
  --iterations 300 --events 100000 --pattern uniform
```

同じdecode済みイベント列から次の3方式を比較する。

|方式|計測内の処理|
|---|---|
|cpu_full_tensor|CPU集計→CPU Tensor生成→全Tensor H2D|
|cpu_bins_gpu_tensor|CPU集計→変更bin H2D→GPU Tensor生成|
|gpu_event_aggregation|イベント列H2D→GPU集計→GPU Tensor生成|

1 frame=4ms相当、shape=[1,20,120,160]、40ms窓。直接実行は既定30回、sweepは既定100回のwarmup後、方式順を3 frameごとに
循環させて比較する。iterationsは3の倍数で10以上。
イベント数は1000/10000/100000/200000 per 4ms、つまり0.25/2.5/25/50 M events/s相当。
uniformと8×8 pixelへ集中するhotspotをsweepする。

計測にはCPU集計、staging、H2D、Tensor生成、GPU完了待ちを含む。
RAW decode、入力配列の生成・確保、USB、ROS、D2H照合、TensorRT推論は含まない。
CPU集計はscalar event()で実行し、ADDR_X runをまとめたdecode経路の計測ではない。
GPU方式もdecode済みイベント配列を入力とするため、本番の配列生成コストを含まない。
今回の比較は集計段階の特性を見るもので、本番3 profileの完全な再現ではない。
各処理を同期して比較するため、非同期overlapの効果やlive latencyは評価しない。
4msを超えた回数は集計stageのbudget超過であり、車両制御のdeadlineを意味しない。
全出力はframeごとにD2Hして全要素照合する。このD2H時間は計測から除外する。

summary.csvのp50/p95/p99とover_4ms_fractionを比較する。
高イベント時には、GPUへのイベント転送が増える一方、bin転送は出力寸法に主に依存する。
ただしCPU集計もイベント数に比例するため、GPUイベント集計が勝つ可能性もある。
イベントが集中した場合のGPU atomicAddの影響も分けて見る。
ノートPCでの勝敗をOrinへ直接外挿しない。最後に実RAW・ROS・推論併走で測定する。
電源状態、power profile、他のGPU workloadを揃え、複数回のrunを残す。

## GPUのないMacで確認できる部分

```bash
cmake -S tools/cuda_benchmark -B build/cpu_checks -DOPENEB_BENCH_CPU_ONLY=ON
cmake --build build/cpu_checks -j 2
ctest --test-dir build/cpu_checks --output-on-failure
python3 tools/cuda_benchmark/sweep.py --binary /not/built --dry-run --repeats 1
```

CPU-onlyではCUDA compiler/driverを探索せず、CPU core/更新binとPython runnerを検証する。
これはCUDA build成功やGPU動作確認を意味しない。

## 3. 実RAWのdecodeからGPU Tensor完成まで比較する（今回の主比較）

このテストでは公式event_camera_codecsのEVT3 decoderと本番のEvt3RunsHistogram、
本番のCUDA backendを直接使う。ROS/OpenEB SDKは不要だが、公式codecのheaderが必要。
本番と同じ版を比較したい場合は、そのcheckoutのincludeを指定する。
以下のclone例は取得時の版を使い、decoder headerのSHA256を結果へ記録する。

```bash
# openeb_ros2のrootで。clone先は未作成directoryを指定する。
git clone https://github.com/ros-event-camera/event_camera_codecs.git /tmp/event_camera_codecs
cmake -S tools/cuda_benchmark -B build/raw_cuda \
  -DCMAKE_BUILD_TYPE=Release -DOPENEB_BENCH_RAW=ON \
  -DEVENT_CAMERA_CODECS_INCLUDE_DIR=/tmp/event_camera_codecs/include
cmake --build build/raw_cuda -j 2
ctest --test-dir build/raw_cuda --output-on-failure

python3 tools/cuda_benchmark/raw_sweep.py \
  --binary build/raw_cuda/event_tensor_raw_cuda_benchmark \
  --raw /absolute/path/to/recording.raw \
  --output results/raw_cuda_01 --repeats 3
```

**このcheckoutの最新ファイルとRAWをLinux PCへコピーしてから実行する。**
Mac上にある変更が別PCへ自動反映されることはない。
既存のdecoded-event用sweep.pyとは実行ファイル・計測範囲が異なる。

|mode|計測する処理|
|---|---|
|events（既存cuda_async）|公式EVT3 decode → 座標LUT/時刻検査 → CudaEvent配列生成 → pinned staging/H2D → GPU集計 → GPU Tensor生成・正規化 → 完了待ち|
|evt3_runs|新EVT3 decode＋CPU直接集計 → CPU Tensor生成・正規化 → 全Tensor H2D → 完了待ち|
|evt3_runs_gpu|新EVT3 decode＋CPU直接集計 → dirty bin抽出/pinned staging/H2D → GPU Tensor生成・正規化 → 完了待ち|

shape=[1,20,120,160]、40ms窓、4ms stride、ON先のpolarity-major、mean=0/std=1。
元解像度はRAWの`% format EVT3;width=...;height=...` headerから読む。
既存方式のvectorは再利用し、必要時の容量増大も計測内。本番と同じ小逆転<=4msはdropする。
大きな逆転はepoch再現の曖昧さを避けて明示的に失敗する。複数epochの性能試験ではない。
既存方式のGPU updateは本番既定の8192イベント単位、vector初期reserveは16384に揃える。
元のUSB packet境界はRAWから復元できないため、共通の偶数byte packetへ分割する。
既定64KiB、`--packet-bytes`で変更できる。frame境界でもpacketを分割し直さない。

RAW読込みと比較窓の事前index作成は計測外で、同じ圧縮byte列を各方式へ入力する。
最初に40ms以上蓄積した窓を作り、その後はpacket終端が4ms境界を越えるたびに
1つの比較窓を作る。窓終端はセンサoriginへ整列し、3方式の時刻は同じ。
packetによる量子化や無イベント区間では処理対象のsensor spanは4msより長くなる。
CSVには実際のspanと入力イベントレートを残す。これは厳密な250Hz timer再現ではない。
末尾の未完成窓は処理せず、loaded/processed_payload_bytesで範囲を確認する。

各frameで方式順を循環させる。各方式の計測末尾でGPU完了を待ち、その後にD2Hして
**全Tensor要素の完全一致・decode採用数・drop数・センサ時刻を照合**する。
D2Hと照合は計測外。Tensor不一致・CUDA errorがあれば失敗し、成功結果を出さない。
GPU命令をenqueueしただけの時間を完了時間と取り違えない。

ただし本番の複数workerのキュー待ち、ROS publisher、USB受信、推論、frame間overlapは除外する。
既存方式では同一frame内で前packetのGPU処理と次packetのCPU decodeが重なる場合がある。
CPU/GPUの初期確保は計測外、イベントvectorのclear/生成とpinnedコピーは計測内。
タイマ計測と公平性確保のための同期・方式切替の影響は含まれる。
`cpu_decode_mean_ms`は既存方式ではdecode＋イベント列生成、新方式ではdecode＋CPU集計。
GPU実行時間のみの内訳ではなく、total_msが主指標。

### 保存結果と負荷別比較

- `report.json`: status、GPU/driver/runtime、codec header hash、入力RAW全体とbinary/source hash、全runの結果。
- `summary.csv`: 各方式のp50/p95/p99のrun間中央値、既存events比、転送量、4ms超過率。
- `windows-0.csv`など: frameごとの入力byte数、イベント数、sensor span、イベントレート、全体時間、decode時間、転送量。warmup行も区別して保存。
- `rate_summary.csv`: <10 / 10..30 / >=30 Mevents/sに分けた全体時間のp50/p95/p99。繰返しrunのsampleをpoolした値で、独立したデータ数ではない。
- stdout/stderr: 失敗も保存。status=completeとall_outputs_identical=trueを確認する。

既定はRAW payload先頭256MiBまでRAMへ読み、最大1000frame、最初30frameをwarmupとする。
`--max-bytes 0 --max-frames 0`で全RAWを扱う（約3GBのRAWならそれ以上の空きRAMが必要）。
wrapperの入力同定hashは常にファイル全体を計算し、この時間も計測外。
全RAWを3回再生する例:

```bash
python3 tools/cuda_benchmark/raw_sweep.py \
  --binary build/raw_cuda/event_tensor_raw_cuda_benchmark \
  --raw /absolute/path/to/recording.raw \
  --max-bytes 0 --max-frames 0 --repeats 3 \
  --output results/raw_cuda_full_01
```

このRAWが高イベントの区間を含まなければ、高負荷での勝敗は判定できない。
syntheticなイベント量別試験は前節のsweep.pyで補い、実RAW結果と区別する。
ノートPCとOrinではGPU、メモリ構成、電力枠が異なるので、Orinでの性能保証にはならない。
Macでは`OPENEB_BENCH_CPU_ONLY=ON`と`OPENEB_BENCH_RAW=ON`を併用すれば、
同じcodec headerに対するRAW index／直接decode／CPU Tensor／変更bin模擬照合を実行できる。
これはCUDAコンパイル・GPU動作の検証ではない。

## 4. CPU中間コピー削減のペア比較

今回の追加は、現在のevt3_runs_gpuを基準に、中間vectorを省く経路を比較するもの。
独立ベンチマークで評価し、現在はROS async nodeのevent_aggregationにも選択肢を追加している。
`evt3_runs_gpu_direct`はこの比較テストとROS parameterの両方で選択できる。

|比較mode|CPUからGPUまでの経路|
|---|---|
|evt3_runs_gpu（基準）|CPU ring → 中間vector → owned pinned staging → GPU ring → GPU 20ch Tensor|
|evt3_runs_gpu_direct（追加）|CPU ring → owned pinned staging → GPU ring → GPU 20ch Tensor|

decode、CPU集計、4ms以下の逆転drop、H2D payload、pinned slot数、GPU kernel、shape、
FP32、正規化、窓終端を揃える。ヒストグラムpayloadのCPUコピー回数だけを2回から1回へ削減。
H2D量は減らさない。GPU ringへの非同期転送は維持し、TensorはGPUメモリ上へ生成する。
CPU copy byte数はmemcpy対象payloadの論理合計で、実際のDRAM bus trafficの計測値ではない。
50%削減はこのコピーbyte数についてであり、全体時間の50%短縮を意味しない。

### 新しいbundleを使う場合

`openeb_raw_cuda_copy_benchmark.tar.gz`をLinuxへコピーして、旧bundleとは別directoryへ展開する。
公式event_camera_codecsのcheckoutとRAWは前の実験で使用したものをそのまま使う。
新旧結果を比較する際は、過去CSVとの比較だけでなく、このbinary内の同時ペア比較を優先する。

```bash
# ~/project/event_camera で実行する例
# event_camera_codecs/ は以前cloneしたdirectoryを使う。
tar -xzf openeb_raw_cuda_copy_benchmark.tar.gz
cd openeb_raw_cuda_copy_benchmark

# 前回導入したCMakeを利用。なければ下記pipコマンドで独立配置する。
# python3 -m pip install --target /tmp/openeb-bench-cmake 'cmake>=3.24,<4'
BENCH_TOOLS=/tmp/openeb-bench-cmake/cmake/data/bin
"$BENCH_TOOLS/cmake" -S tools/cuda_benchmark -B build/raw_copy_cuda \
  -DCMAKE_BUILD_TYPE=Release -DOPENEB_BENCH_RAW=ON \
  -DEVENT_CAMERA_CODECS_INCLUDE_DIR="$PWD/../event_camera_codecs/include" &&
"$BENCH_TOOLS/cmake" --build build/raw_copy_cuda -j 2 &&
"$BENCH_TOOLS/ctest" --test-dir build/raw_copy_cuda --output-on-failure
```

CMake 3.24以上が通常PATHにある場合は、cmake/ctestを直接使ってよい。
全テストが通った後、前回と同じRAW、同じ256MiB/1000frame/30warmup条件で実行する:

```bash
RAW="/home/aten-22/project/event_camera/openeb_camera_20261021_184014_000000.raw"
python3 tools/cuda_benchmark/copy_sweep.py \
  --binary build/raw_copy_cuda/event_tensor_raw_copy_cuda_benchmark \
  --raw "$RAW" --output results/raw_copy_01 --repeats 3
cat results/raw_copy_01/summary.csv
cat results/raw_copy_01/rate_summary.csv
```

`--max-bytes 0 --max-frames 0`を追加すると全RAWを処理する。
出力directoryは未作成名を指定し、前の結果を上書きしない。

### 計測と一致確認

- RAW decode → CPU集計 → bin抽出 → pinned stagingへのコピー → H2D → GPU Tensor生成 → GPU完了待ちをtotal_msへ含める。
- 初期確保・RAW読込み/index・D2H照合・CPU参照Tensorの生成は計測外。
- 方式順をframeごとに交互にし、run開始順も変更。各方式は独立したCPU/GPU ringを持つ。
- 毎窓、GPU Tensor同士、およびGPU TensorとCPU参照の全1×20×120×160要素をbitwise比較する。
- H2D payload、decode採用/drop数、センサ時刻も照合。不一致があれば失敗する。
- 非単位正規化、polarity/time-major、signed/separate、ring wrap/clear、epoch reset、metadata-only更新はCUDA correctness testでも比較する。
- CPU更新はpinned stagingへのコピー完了後に再開できる。H2D完了前にCPU ringを変更してもGPU結果が変わらないことをCUDA testで検査する。

`summary.csv`の`speedup_vs_evt3_runs_gpu_p50/p95/p99`は1超で基準より速い。
`cpu_export_mean_ms_median`は中間vectorコピーまたはborrowed view作成の時間、
`gpu_submit_mean_ms_median`はpinnedコピー＋CUDA enqueue＋snapshot起動のhost wall time、
`gpu_wait_mean_ms_median`は最後のstream完了待ち。GPU純実行時間やCPU使用率ではない。
`histogram_cpu_copy_bytes_median`は新方式で基準の半分、`h2d_bytes_median`は同量になる。
`windows-*.csv`にはこの内訳を各窓について保存する。
report.jsonのcomparison=histogram_cpu_copy、tensor_shape=[1,20,120,160]、
all_outputs_identical=true、cpu_reference_identical=true、status=completeを確認する。
従来raw_sweep.pyや古いbinaryを誤指定した場合はschema検査で失敗する。

### APIの寿命・排他条件

`Evt3RunsHistogram::peek_dirty_bins(HistogramBinView&)`はpixel配列をコピーせず、
ringのplaneポインタとmetadataだけを参照する。dirty flagは消さない。
呼出側はpeekから`EventTensorCudaBackend::update_bins_direct()`終了までhistogramの変更を禁止する。
ROSではこの区間を同一mutexで保護する。pinned slot再利用待ちが
lock保持時間を延ばす可能性も含めて別途評価する。独立ベンチマークは単一threadで再生する。
backendは既存と同じ4個のowned pinned slotへ同期CPUコピーを行ってから非同期H2Dをenqueueし、
slot再利用時はCUDA eventを待つ。CUDAへCPU ringの借用ポインタを直接渡さない。
成功後のみ`acknowledge_dirty_bins()`し、失敗時はdirtyを保持する。
backendがepochを無効化した場合はmark_all_dirty()して全slotを再送する。
CPU-only testはborrowed view→owned stagingの一致・寿命分離・wrap/resetを検査する。
このMacでのCPU検証はCUDA build成功やGPU速度の証明ではない。

## 5. 圧縮イベント群をGPUでヒストグラム化するペア比較

`evt3_runs_gpu_direct`（CPU集計＋直接pinnedコピー）を基準に、
新しい`evt3_runs_grouped_gpu`（CPU状態復元＋GPU集計）を同一binary内で比較する。
両方ともdecode開始からGPU上のFP32 [1,20,120,160] Tensor完成待ちまでをtotal_msへ含める。
現在はROS async nodeの選択肢にも追加。既定eventsは維持。ROS liveの性能は独立benchmarkと区別する。

|mode|CPU側|H2D|GPU側|
|---|---|---|---|
|evt3_runs_gpu_direct|EVT3 decode＋pixel histogram集計|変更bin|20ch Tensor構成・正規化|
|evt3_runs_grouped_gpu|EVT3状態復元、座標範囲・逆転検査、run descriptor生成|圧縮RAW＋descriptor|ヒストグラム集計＋同じ20ch Tensor構成・正規化|

### 実行

新しい`openeb_raw_cuda_grouped_benchmark.tar.gz`を前のbundleと同じ親directoryへコピーする。
既存のevent_camera_codecs checkout、RAW、CMakeを再利用する。

```bash
cd ~/project/event_camera
tar -xzf openeb_raw_cuda_grouped_benchmark.tar.gz
cd openeb_raw_cuda_grouped_benchmark
BENCH_TOOLS=/tmp/openeb-bench-cmake/cmake/data/bin
# 上記CMakeがなければ独立配置:
# python3 -m pip install --target /tmp/openeb-bench-cmake 'cmake>=3.24,<4'
"$BENCH_TOOLS/cmake" -S tools/cuda_benchmark -B build/raw_grouped_cuda \
  -DCMAKE_BUILD_TYPE=Release -DOPENEB_BENCH_RAW=ON \
  -DEVENT_CAMERA_CODECS_INCLUDE_DIR="$PWD/../event_camera_codecs/include" &&
"$BENCH_TOOLS/cmake" --build build/raw_grouped_cuda -j 2 &&
"$BENCH_TOOLS/ctest" --test-dir build/raw_grouped_cuda --output-on-failure
```

テスト通過後に実行する:

```bash
RAW="/home/aten-22/project/event_camera/openeb_camera_20261021_184014_000000.raw"
python3 tools/cuda_benchmark/grouped_sweep.py \
  --binary build/raw_grouped_cuda/event_tensor_raw_grouped_cuda_benchmark \
  --raw "$RAW" --output results/raw_grouped_01 --repeats 3
cat results/raw_grouped_01/summary.csv
cat results/raw_grouped_01/rate_summary.csv
```

既定の先頭256MiB・最大1000frame・30warmup、4ms stride/40ms window、ON先polarity-majorを維持。
全RAWなら`--max-bytes 0 --max-frames 0`を追加する。CPUのRAWプリロード用RAMが必要。
同じbinary内で方式順をframeごとに交互にし、各runの開始順も交互にする。
旧CSVとの単純比較より、このペア比較を優先する。

### 圧縮表現と処理範囲

CPUのEvt3RunScannerは時刻・Y・vector baseなどの状態を復元するが、pixel histogramを持たない。
連続するADDR_Xワードを1 runにまとめる。24byteのdescriptorはbin、RAW内offset/word数、
縮小後Y、vector base、種別、極性を持つ。ADDR_XのX/極性は元16bit wordのままGPUが読む。
VECT12/VECT8は1 encoded wordに1 descriptorを付け、bit展開と加算をGPUで行う。
CPUで展開済み16byteイベント配列を作らず、GPU上にも展開済みイベント配列を作らない。
GPUは1 warp/run、1 block内4 runで処理し、既存と同じrolling ringへatomic加算する。
元RAWの制御ワードも転送するため、payloadはRAW byte数＋24byte×run数。
runが短い場合・vector wordが多い場合はdescriptorが増え、速度向上を保証しない。
CPUは正しいoriginと逆転dropを保持するためにX範囲も検査する。完全GPU decodeではない。

新旧ともhalf-open窓、floor縮小、ON/OFF順、4ms以下逆転dropを維持する。
scannerはHIGH/LOW取得条件・HIGH rollover・uint16 vector base wrapを既存decodeと揃える。
単体testでは大きな逆転によるepoch resetも検査するが、RAW benchmark自体は従来同様に
公式decoderでindexできる単一epochを対象とし、4ms超の逆転はindex段階で拒否する。

RAWと比較窓のindex、必要staging容量を決める追加scanner pass、初期確保は計測外。
このprepassは最大RAW frame長/run数を求めるだけで、計測時には再度すべてscanしてdescriptorを生成する。
2個のpinned/device staging slotを事前確保し、timed loop中の拡張は行わない。
result JSONにgrouped_raw_capacity_bytes/run_capacity/staging_slotsを残す。
毎frameのRAWコピー・descriptorコピー・H2D・GPU clear/集計・snapshotは計測内。
GPU完了前の入力再利用はCUDA eventで保護する。CPU pointerをkernelへ直接渡さない。

毎窓、2方式のGPU Tensorを全要素bitwise照合し、CPU参照Tensorとも照合する。
採用/drop/OOB数・origin/latest/epochも確認する。不一致は失敗しcompleteを出さない。
CPU参照Tensor生成、D2H、比較は計測外。USB、ROS待ち/publication、推論、frame間overlapも除外する。
packet境界は共通64KiBの模擬境界で、元USB packetの再現ではない。
frame内に共通した端数やgapがあるため、CSVのsensor_span_msも合わせて読む。

### 結果

`summary.csv`のspeedup_vs_evt3_runs_gpu_direct_p50/p95/p99は、1超で新方式が速い。
cpu_decode_mean_ms_medianは、基準ではdecode＋CPU histogram、新方式では状態復元・検査・run生成。
gpu_submit_mean_ms_medianはhost側のstagingコピー・検査・CUDA起動、gpu_wait_mean_ms_medianは完了待ち。
GPU純実行時間、CPU使用率とは区別する。主判定はdecodeからGPU完了までのtotal_ms。
`h2d_bytes_median`に加え、新方式のcompressed_raw_h2d_bytes_median、run_descriptor_h2d_bytes_median、
runs_transferred_medianを記録し、転送量増減も確認できる。
`windows-*.csv`と`rate_summary.csv`で高イベント時間のp95/p99を比較する。
report.jsonはcomparison=compressed_gpu_histogram、status=complete、all_outputs_identical=true、
cpu_reference_identical=true、tensor_shape=[1,20,120,160]を確認する。
古いcopy benchmarkを誤指定した場合はschema検査で失敗する。

### 確認範囲

CPU-onlyでscannerと圧縮run集計の模擬処理を検査し、既存core・公式decoderの参照結果と照合する。
合成テストはpacket分割、HIGH/LOW取得、HIGH rollover、vector mask/base wrap、OOB、小逆転、
大逆転reset、signed/separate、layout、非単位正規化、rolling ringの古いbin消去を含む。
Linuxのevt3_runs_grouped_cuda_testは同じ結果を実GPUと比較し、2 staging slotの再利用と
update後のRAW破壊・再利用でGPU入力が変わらないことも検査する。
Macのportable検査やCPU模擬の一致は、nvcc compile・GPU一致・高速化を確認した意味ではない。
OrinやROSでの実運用性能は、Linux notebookでの結果とも分けて確認する。
