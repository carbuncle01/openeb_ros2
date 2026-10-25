# openeb_ros2 — kart branch

OpenEB/SilkyEvCamの取得、native RAW保存、イベント画像、イベントtensor生成を担うsensor package。
モデル推論・学習・車両制御は利用側が所有する。
新しいtensor経路はROS packet topicを入力にせず、OpenEBの取得bufferを同一プロセス内で直接渡す。

対象は **Isaac ROS 5.0 / ROS 2 Lyrical / CUDA / SilkyEvCam対応OpenEB**。
Jazzy/旧NITROS向けjetpilot-v1.0.0とは出力APIが異なる。

## 3方式

| tensor_backend | 実装 | 処理 |
|---|---|---|
| cpu | EventTensorEncoderNode | CPU full/incremental表現 → FP32 tensor全体をH2D |
| cuda | EventTensorEncoderNode | 新着eventをH2D → GPU rolling histogram → snapshot |
| cuda_async（基準値） | AsyncEventTensorPreprocessorNode | 有限長packet queue → decode worker → GPU worker → 期限を優先したsnapshot |

3方式はEVT3 decoderの違いではなく、テンソル表現の生成方式の違い。
CPU／従来CUDAは取得callback内で処理する。低遅延運用候補はcuda_async。

```text
OpenEB RAW callback
  → 所有bufferへコピー（SDK callbackごと。追加packet集約なし）
  → direct ingest（ROS publish/subscribeなし）
  → cpu / cuda / cuda_async
  → CUDA-backed TensorList [1,C,H,W]
  → Isaac ROS TensorRT

監視がONかつ購読者ありの時だけ分岐 → events_raw
native RAW保存はOpenEB側で独立
```

## ビルド

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select openeb_ros2
source install/setup.bash
```

OpenEB/Metavisionのbase/core/stream/HAL、event_camera_msgs、event_camera_codecs、
Isaac ROS common/tensor_msgs、tensor_msgs、cuda_buffer/backend、CUDA toolkitが必要。
`ament_cmake_auto`を使用する。OPENEB_BUILD_TENSOR_PIPELINE=OFFではtensor対象とCUDA言語の
ビルドを省略するが、package.xmlで宣言したROS依存は必要。
このbranchはjetpilot_msgsへ依存しない。旧BagRequest入力は廃止し、RAW記録は下記サービスを使う。

## 起動と設定

```bash
ros2 launch openeb_ros2 tensor.launch.py tensor_backend:=cuda_async packet_publish_enabled:=false
# 比較用にtensor_backend:=cpu または cuda
```

全parameterの基準値はconfig/にあり、[parameter一覧](docs/parameters.md)に既定値と意味を記載。
利用側bringupの設定を使う場合はdriver_config/tensor_config/pipeline_configへその絶対パスを渡す。
launchは静的parameterを重複定義せず、明示された引数だけ上書きする。

| launch引数 | 既定 | 意味 |
|---|---|---|
| namespace | event_camera | 起動する全nodeのnamespace |
| pipeline_config | packageのconfig/tensor_pipeline.yaml | tensor_backendの正本 |
| driver_config | packageのconfig/driver.yaml | driver全設定 |
| tensor_config | 空 | 選択backendのconfig/tensor_<backend>.yamlを使用 |
| image_config | packageのconfig/visualization.yaml | 可視化の全設定。既定は白背景・ON赤・OFF青 |
| tensor_backend | 空 | configを保持。明示時cpu/cuda/cuda_async |
| packet_publish_enabled | 空 | configを保持。明示時true/false |
| serial | 空 | configを保持。カメラserialの明示上書き |

負荷が大きい時はpacket配信だけ停止できる。

```bash
ros2 param set /event_camera/event_camera_driver packet_publish_enabled false
ros2 param set /event_camera/event_camera_driver packet_publish_enabled true
```

OFFでもtensor生成・native RAW記録は継続する。trueでも購読者がいなければ配信しない。
自動で負荷を判断してOFFにする処理はない。backendの変更は再起動が必要。RAW保存先raw_recording_dirは停止中に絶対パスへ変更できる。
停止指示の直前に開始済みのpacket処理は完了し得るため、OFFで既に配信済みのqueueを消去するわけではない。

## ノード・topic

| node（既定namespace） | 実行ファイル / plugin | 役割 |
|---|---|---|
| /event_camera/tensor_pipeline | openeb_tensor_pipeline | backend選択。独自入出力なし |
| /event_camera/event_camera_driver | 同上に内蔵 / openeb_ros2::DriverComponent | カメラ所有、direct入力、任意packet配信、RAW保存 |
| /event_camera/event_tensor | 同上に内蔵 / openeb_ros2::EventTensorEncoderNodeまたはAsyncEventTensorPreprocessorNode | tensor生成 |
| /event_camera/tensor_container | 同上に内蔵 | TensorRT等をloadするcomponent manager |
| /event_camera/event_preprocessor | 内蔵版またはopeneb_preprocessor_node / openeb_ros2::PreprocessorComponent | direct可視化または既存のpacket検証・画像化 |

単体driver実行ファイルはopeneb_driver_node（直接実行時node名openeb_driver）。
単体tensor実行ファイルはevent_tensor_encoder_node / async_event_tensor_preprocessor_node
（直接実行時node名event_tensor_encoder / async_event_tensor_preprocessor）。
単体preprocessorの直接実行時node名はopeneb_preprocessor。
単体tensor componentは既定subscribe_packets=trueでeventsを購読する。内蔵版では強制false。
以下はremapなし・namespace=event_cameraでの既定名。全て相対topicなのでnamespace/remapで変更可能。

| node | 方向 | topic | 型 | QoS | 内容 |
|---|---|---|---|---|---|
| driver | 出力 | /event_camera/events_raw | event_camera_msgs/msg/EventPacket | best effort / volatile / depth 8 | 配信ON＋購読者あり時だけRAW EVT3 |
| driver | 出力 | /event_camera/diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable / volatile / depth 1 | 既定1 Hzの取得・配信・記録診断 |
| tensor | 出力 | /event_camera/tensor | isaac_ros_tensor_msgs/msg/TensorList | reliable / volatile / cpu,cuda: depth 4; async: depth 8 | GPU-backed FP32 NCHW。既定250 Hz |
| tensor | 出力 | /event_camera/tensor_diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable / volatile / depth 10 | 既定1 Hz、queue滞留・decode/GPU処理・deadline等 |
| tensor（単体のみ） | 入力 | /event_camera/events | event_camera_msgs/msg/EventPacket | best effort / volatile / legacy: depth 8; async: depth 16 | topic入力を選んだ場合 |
| 従来CUDA（consumer_driven時） | 入力 | /event_camera/tensor_feedback | isaac_ros_tensor_msgs/msg/TensorList | reliable / volatile / depth 1 | 同一stampの推論完了通知 |
| preprocessor（単体のみ） | 入力 | /event_camera/events_raw | event_camera_msgs/msg/EventPacket | best effort / volatile / depth 8 | RAW入力 |
| preprocessor（packet forwarding有効時） | 出力 | /event_camera/events | event_camera_msgs/msg/EventPacket | best effort / volatile / depth 8 | 検証済みpacket |
| preprocessor | 出力 | /event_camera/event_image | sensor_msgs/msg/Image | best effort / volatile / depth 2 | 画像有効＋購読者あり時。direct profileはbgr8、25 Hz、ON赤/OFF青/背景白 |
| preprocessor | 出力 | /event_camera/diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable / volatile / depth 1 | 既定1 Hz診断 |

ROS標準のrosout/parameter_events等は一覧から省略。

## NCHWとzero-copy

既定shapeは **[1,20,120,160]**（10 bins × ON/OFF）、contiguous FP32。
モデルの入力名、shape、window、stride、極性順、正規化を一致させる。
CUDA方式ではsnapshot kernelがCUDA-backed ExperimentalTensor.dataへ直接書き、
WriteHandleを閉じてから標準TensorListをpublishする。CPU方式はH2Dが必要。

Isaac ROS 5.0では旧NitrosTensorListではなくrosidl::Buffer＋CUDA backendを使う。
TensorRTを同一processへloadする場合のmanagerは/event_camera/tensor_container。
利用側launchのLoadComposableNodes(target_container="/event_camera/tensor_container", ...)
でTensorRTを追加し、入力tensor_subを/event_camera/tensorへremapする。
起動順はmanagerのload serviceが利用可能になってから。CUDA stream間の同期はbuffer handleが担う。

公式仕様上、同一host/GPU/userかつVMM対応のendpoint間ではGPU payload共有が可能。
条件外はCPU転送へfallbackする。実環境でzero-copyが成立するか、TensorRT内の追加コピー、
E2E遅延、250 Hz達成は別途実測が必要。SDK入力コピーとeventのH2Dまで消えるという意味ではない。
[公式CUDA backend](https://nvidia-isaac-ros.github.io/concepts/rosidl_buffer/cuda_buffer_backend.html)

## native RAW保存と画像

raw_recording_enabled=trueで保存を有効化する。単独記録時はraw_recording_dir=/絶対パスを設定する。
raw_recording_auto_start=falseなら明示開始する。連携待機時は保存先を空で起動でき、停止中に動的設定する。

```bash
ros2 service call /event_camera/start_raw_recording std_srvs/srv/Trigger '{}'
ros2 service call /event_camera/stop_raw_recording std_srvs/srv/Trigger '{}'
ros2 service call /event_camera/split_raw_recording std_srvs/srv/Trigger '{}'
```

同名サービスはdriver namespace相対、標準Trigger。0より大きいraw_recording_split_duration_sで自動分割。
RAWと時刻metadata sidecarを保存する。RGBとの厳密な同期はこのmetadataだけでは保証しない。
既存driver.launch.py / pipeline.launch.py / composed.launch.pyはpacket・画像用で残している。
tensor.launch.pyでは画像用preprocessorも同一processに起動する。
packet_forward_enabled=false / subscribe_packets=falseで、packet topic配信OFFでも画像を出力できる。
監視購読者がある時だけ別workerで追加decodeし、queueは4 packet / 8 MiB / 100 msに制限する。
queue overflow・古いpacket破棄後は画像decoderをresetする。tensor入力queueとは独立。
red_blue_white方式では40 ms窓、40 ms更新（25 Hz）、空窓を白画像で配信する。
ON/OFF両方あるpixelは正規化強度の大きい極性を表示し、同値ならON赤。
画像stampはpacket受信時刻へ対応づけた推定ROS時刻で、hardware同期は保証しない。
画像化の追加CPU負荷・コピーは残るため、不要ならimage_configのevent_image_enabled=falseで起動する。

## 検証

```bash
# ROS/GPU不要: 配信ON/OFF、direct継続、monitorだけ追加copy、順序保持
clang++ -std=c++17 -Wall -Wextra -Werror -I include test/packet_router_test.cpp -o /tmp/openeb_packet_router_test
/tmp/openeb_packet_router_test

# 対象ROS/GPU環境: CUDA histogram、channel順、window更新等
colcon test --packages-select openeb_ros2
colcon test-result --verbose
```

ローカルmacOSではpacket routingのC++テスト、launch構文、parameter/config整合を確認。
ROS・OpenEB・CUDAビルド、カメラ取得、TensorRT結合、zero-copy・実測遅延は未確認。
GPUテストを移植済みだがローカルでは未実行。[検証記録](docs/validation.md)参照。
移植元・設計契約は[方針](docs/policies/tensor_pipeline.md)に記載。


## kart bag managerとの連携

kart_bag_managerのraw_recording_driver_nodeを/event_camera/event_camera_driverへ設定する。
managerがrosbag出力directoryの生成を確認した後、標準SetParametersAtomicallyで保存先を設定し、
成功応答を待ってTrigger STARTを呼ぶ。sensor packageはkart_interfacesへ依存しない。

```bash
ros2 launch openeb_ros2 tensor.launch.py \
  driver_config:="$(ros2 pkg prefix openeb_ros2)/share/openeb_ros2/config/driver_bag_linked.yaml"
```

driver_bag_linked.yamlは全driver parameterを記載した待機用profile。
RAW保存を有効にし、auto_startとdriver独立split timerを無効にし、packet配信はOFF。
manager側はkart_bringup/config/recording/bag_manager_openeb.yamlを運用設定として使う。
同じfilesystem/mountの同じsession directoryにMCAP、RAW、RAW時刻metadataを保存する。
RAW開始・停止の成功はmanagerのbag/raw_diagnosticsとdriver diagnosticsで別途確認する。
RAW失敗だけではrosbagを停止しない。MCAP/RAWの分割境界・sensor同期は別途測定する。

RAWだけ録画することもできる。連携待機profileで起動し、停止中に保存先を指定して開始する。

```bash
ros2 param set /event_camera/event_camera_driver raw_recording_dir /workspaces/record/raw_only_01
ros2 service call /event_camera/start_raw_recording std_srvs/srv/Trigger '{}'
```

手動RAWがactiveな間の保存先変更は拒否する。bag managerの開始要求がそのために失敗した場合、
managerは手動RAW writerを停止しない。

## 学習用の時刻メタデータ

3方式ともTensor出版後に`/event_camera/tensor_timing`（std_msgs/msg/String、JSON v1）を
出版する。reliable/volatile/depth 512、Tensorと同周期（既定250 Hz）、購読者あり時のみ。
追加parameterなし。GPU Tensor本体は記録不要。OpenEB用bag profileはこのtopicを記録する。
JSONはstream_id、sequence、sensor_epoch、tensor_stamp_ros_ns、generated_ros_ns、
window_start/end_sensor_us、sensor_to_ros_offset_ns、window_start/end_ros_ns、
latest_event_sensor_us、tensor_name、frame_id、shapeを含む。
窓は半開区間[start,end)。header stampと窓終端は同一とは限らない。
sequenceは成功したTensor出版ごとに増加、リセット時sensor_epochが増える。
メタデータとTensorの出版は別メッセージなのでatomicではなく、欠落はsequenceで確認する。

offsetは毎snapshotのpacket受信ROS時刻−最新イベントセンサ時刻から得る推定値。
USB遅延/queue遅延とclock変動が入り、hardware sync精度を保証しない。
RAWの0始まりはreaderのtime shiftingに依存する。recording_start_ros_timeを
RAWの0秒へ直接足さない。HALのget_timestamp_shift()でファイルごとのshiftを取得し、
`ros_ns=(raw_relative_us+raw_sensor_shift_us)*1000+sensor_to_ros_offset_ns`で対応付ける。
RAW splitごとにshiftを取得し、timestamp reset/rolloverはepoch別に扱う。
SDK実装: https://github.com/prophesee-ai/openeb/blob/5.2.0/hal/cpp/include/metavision/hal/facilities/i_events_stream_decoder.h

## cuda_asyncの直接集計（opt-in）

`event_aggregation`（string、既定`events`、起動時固定）で選択する。
`events`は従来のevent配列・GPU ring方式。`evt3_runs`は受信済みEVT3 byte列の
連続ADDR_Xで時刻・行・binの準備を共有し、CPU ringへ直接加算する。
VECT12/VECT8も処理する。イベント配列・event staging・event H2Dは作らない。
SDK bufferから所有するRAW byte bufferへのコピーは維持し、workerは所有bufferを
処理完了まで保持する。queue容量・年齢上限・停止joinは既存のものを使用する。

既存`openeb_ros2::AsyncEventTensorPreprocessorNode`、実行ファイル
`async_event_tensor_preprocessor_node`、既定単体node名`async_event_tensor_preprocessor`を使用。
統合起動時のnode名`/event_camera/event_tensor`と入出力は既存と同じ。
`/event_camera/tensor`はisaac_ros_tensor_msgs/msg/TensorList（reliable/volatile/depth 8）、
`/event_camera/tensor_timing`はstd_msgs/msg/String（reliable/volatile/depth 512）、
`/event_camera/tensor_diagnostics`はdiagnostic_msgs/msg/DiagnosticArray（depth 10）。
単体でsubscribe_packets=trueの場合のみevents（event_camera_msgs/msg/EventPacket、
best effort/volatile/depth 16）を購読。direct pipelineでは購読を強制OFFにする。

```bash
ros2 launch openeb_ros2 tensor.launch.py tensor_backend:=cuda_async \
  tensor_config:="$(ros2 pkg prefix openeb_ros2)/share/openeb_ros2/config/tensor_cuda_async_runs.yaml"
```

通常profileの既定はeventsを維持する。runs profileは全parameterを保持し、
event_aggregationだけをevt3_runsへ変更したもの。変更には再起動が必要。
channel/polarity順、半開時間窓、空binの減衰、shape、正規化、TensorTimingを維持する。
初期時刻確定・24bit rollover・小さいTIME_HIGH逆行の扱いは
[event_camera_codecs](https://github.com/ros-event-camera/event_camera_codecs/blob/master/include/event_camera_codecs/evt3_decoder.h)
に合わせる。packetは偶数byteのEVT3限定。他encoding・未知code・不正packetはエラーにして
stateをresetし、自動で別decoderへ切り替えない。trigger/monitorはTensorへ含めない。
seq欠落/逆順、geometry変更、decoderエラーはparser/ringをresetし、
次の有効時刻から新epochとして再開する。小さいCD時刻逆行は許容閾値まで破棄する。

この初版は完成FP32 Tensor全体をGPUへ転送し、転送完了後にCPU snapshot bufferを
再利用する。既定shape・250Hzでは約384 MB/sのpayload。更新binだけのGPU転送は未実装。
CPU decodeとsnapshotのstate参照はmutexで保護する。packet処理がsnapshotを待たせる
可能性があり、Macでの単独algorithm速度をROS/Orinの周期保証と解釈しない。
診断event_aggregation/tensor_uploadで採用方式を確認する。decoded queue/poolと
GPU event transferに関する既存parameter・診断はevt3_runsでは未使用。
実データの出力比較は同じsensor epoch・window_start/endで行う。異なる負荷条件下の
wall timerによる出版回数・header・欠落の一致まではalgorithmの一致試験に含めない。

```bash
# ROS/CUDA不要のcore試験
c++ -std=c++17 -O2 -Wall -Wextra -Werror -I include \
  test/evt3_runs_histogram_test.cpp -o /tmp/evt3_runs_histogram_test
/tmp/evt3_runs_histogram_test
# installed event_camera_codecsとの合成照合、およびGPU集計との照合はcolcon testへ登録
colcon test --packages-select openeb_ros2
colcon test-result --verbose
# build直下の比較ツールは全RAWの全Tensorをmemcmpで照合できる
./build/openeb_ros2/evt3_runs_codec_equivalence_test --raw /absolute/path/camera.raw
```

Macで公式decoderとの全RAW照合とcore単体試験を実施。ROS/SDK/CUDAビルド、GPU比較、
USBカメラ、TensorRT接続、Orin性能は未確認。[検証記録](docs/validation.md)を参照。

## 直接集計とGPU Tensor生成の併用

`event_aggregation=evt3_runs_gpu`（起動時固定）を追加した。既定eventsとCPU版evt3_runsは維持。
同じAsyncEventTensorPreprocessorNode、統合node /event_camera/event_tensor、既存入出力topic、
QoS、shape、時刻窓、正規化parameterを使う。新しいROS node/topic/依存は追加しない。

```text
所有RAW buffer → CPUでEVT3 decode・bin集計
             → 変更binの絶対値だけ転送
             → GPU ring → CUDAで20ch Tensor構成・正規化 → TensorRT
```

```bash
ros2 launch openeb_ros2 tensor.launch.py tensor_backend:=cuda_async \
  tensor_config:="$(ros2 pkg prefix openeb_ros2)/share/openeb_ros2/config/tensor_cuda_async_runs_gpu.yaml"
```

新profileは全parameterを保持し、event_aggregationだけをevt3_runs_gpuにする。
診断はevent_aggregation=evt3_runs_gpu / tensor_upload=dirty_cpu_binsを出す。
CPU側は変更された時間slotを追跡し、読み取り中のbinも含めて必要なslotだけを取得する。
ON/OFF planeを含むbinの絶対値を上書きするので、同じslotを再送しても二重加算しない。
空binを作るためのclearも変更として転送。初回、epoch変更、大きい時刻jump、失敗後のretryは
必要な全slotを再送する。小さい時刻逆行の許容・破棄方針（既定4ms）は変更しない。

転送データはCPU ringから所有する更新bufferへコピーし、mutexを解放する。
CUDA backendは4個の固定pinned bufferへbinをコピーし、完了eventを確認して再利用する。
GPU ringへの非同期copyとsnapshot kernelは同じstreamにenqueueする。全TensorのH2Dも、
event列のH2Dも行わない。GPUは既存make_snapshot kernelで画素/channelごとに並列処理する。
更新binのsnapshotがCPUで完了した時点のstateを出版する。CPUの後続更新は次回に反映する。

## GPU版の検証と比較

MacでCUDA実行はできないため、bin export/絶対上書き/再構成のCPU模擬と公式decoderを比較した。
全RAWの13,272 Tensorが全要素一致。初回全送信、変更なし、途中binの再更新、所有buffer寿命、
同じupdateの再送、ring周回、空窓、epoch reset、失敗後の全再送、signed/layoutを単体確認。
同一64KiB packet列・4ms窓境界でのH2D対象payloadは、全Tensor 20,385,792,000 byteに対し
変更bin 4,065,177,600 byte（約80.1%減）。実際のUSB到着・出版時刻・欠落で転送量は変わる。
これは転送対象データ量の計算であり、CUDA帯域や実行時間の測定ではない。

Orin/対応CUDA環境で、ros2_wsから実行する:

```bash
colcon build --packages-select openeb_ros2 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select openeb_ros2
colcon test-result --verbose
./build/openeb_ros2/histogram_bin_cuda_benchmark --iterations 1000 --events 100000
```

histogram_bin_cuda_testは実GPUでCPU出力との一致とpinned bufferの再利用、時刻/epoch更新を比較する。
benchmarkは共通の合成CPU histogramを入力に、全Tensor H2Dと変更bin H2D＋GPU生成を比較する。
100000 event/4ms、100回warmup、方式順を交互にし、mean/p50/p95/p99とpayload byteをJSONへ出す。
計測はsnapshot準備開始からcudaStreamSynchronize完了まで。decode・USB・入力生成・D2H照合は除外。
各処理を同期させる比較で、実運用の非同期overlapやTensorRT併走の速度保証ではない。
MacではこのCUDA test/benchmarkは未実行、ROS/SDK/CUDA buildも未確認。
本番評価では3つのprofileを同一条件で別runし、drop、sensor age、GPU負荷、TensorRT併走を確認する。
既存診断snapshot_msは主にhostのenqueue時間であり、非同期GPU完了時間ではない。

## Jetson以外のCUDAノートPCで比較

ROS/OpenEB SDK不要の独立CMake buildとイベント量別3方式比較を
[tools/cuda_benchmark](tools/cuda_benchmark/README.md)へ追加した。
本番と同じCUDA backendを使い、全Tensor転送・変更bin転送＋GPU構成・GPUイベント集計を比較する。
全イベント量でGPUが速いという前提は置かない。合成decode済みイベントの集計stageを測り、
RAW decode・配列生成・USB・ROS・推論は除外する。常駐メモリとH2D payloadを区別する。
ROS node/topic/parameterは追加なし。MacでCPU-onlyのbuild/testを確認、CUDA実行は未確認。

### RAW decodeからTensor完成までをLinux NVIDIA PCで比較

[単独CUDAテストの実RAW比較手順](tools/cuda_benchmark/README.md#3-実rawのdecodeからgpu-tensor完成まで比較する今回の主比較)
を追加。`OPENEB_BENCH_RAW=ON`で公式event_camera_codecs headerを指定し、
`event_tensor_raw_cuda_benchmark`と`raw_sweep.py`を使う。
既存events／evt3_runs／evt3_runs_gpuへ同じRAW packet・窓終端を与え、
decode、イベント列生成または直接集計、転送、Tensor生成、GPU完了待ちを含めて比較する。
全Tensor要素の一致を毎窓確認し、p50/p95/p99、イベントレート別・窓別CSVを保存する。
USB/ROS queue/publication/推論とframe間overlapは含まない。通常node/configは変更しない。
MacのCPU照合とCUDA host側の構文検査まで実施。CUDA build・GPU速度はLinux実行で要確認。

### CPU中間コピー削減の比較

`EventTensorCudaBackend::update_bins_direct(HistogramBinView)`と
`Evt3RunsHistogram::peek_dirty_bins()/acknowledge_dirty_bins()`を追加した。
CPU ring→中間vector→pinned stagingのうち中間vectorを省き、GPU転送量・FP32 20ch Tensorを保持する。
[Linux比較手順](tools/cuda_benchmark/README.md#4-cpu中間コピー削減のペア比較)の
`event_tensor_raw_copy_cuda_benchmark`／`copy_sweep.py`で現在のevt3_runs_gpuとペア比較する。
evt3_runs_gpu_directは比較用に加え、現在はROS async nodeの起動時選択にも対応。既定eventsとtopicは保持。
毎窓GPU同士とCPU参照のbitwise一致・H2D量一致を検査し、copy byte数と処理時間内訳を保存する。
MacではCPU portable検査と実RAW冒頭100窓の一致、CUDA host C++構文検査を実施。
CUDA compile、GPU一致確認、高速化はLinuxで実行する必要がある。

### 圧縮EVT3 runのGPU集計を比較

`Evt3RunScanner`でCPUのpixel集計を省き、元の16bit RAW＋24byte/run descriptorを
`EventTensorCudaBackend::update_grouped_raw()`へ転送してGPUでヒストグラムを生成する方式を追加。
展開済みイベント配列はCPU/GPUとも作らず、既存のsnapshot kernelでFP32 20ch Tensorを生成する。
[Linux比較手順](tools/cuda_benchmark/README.md#5-圧縮イベント群をgpuでヒストグラム化するペア比較)の
`event_tensor_raw_grouped_cuda_benchmark`／`grouped_sweep.py`でevt3_runs_gpu_directとペア比較する。
毎窓、GPU Tensor同士＋CPU参照のbitwise一致とdecode数・時刻を確認し、p50/p95/p99とRAW/descriptor転送量を保存。
evt3_runs_grouped_gpuは比較用に加え、現在はROS async nodeの起動時選択にも対応。既存方式とtopicは保持。
Macのportable検査とCPU模擬照合を実施。CUDA compile・GPU一致・速度はLinux実行で確認する。


### ROSでGPU run集計を選択する

AsyncEventTensorPreprocessorNodeのevent_aggregationにevt3_runs_gpu_direct / evt3_runs_grouped_gpuを追加。
既定eventsと既存evt3_runs / evt3_runs_gpuは維持する。全方式startup-only。
node/plugin/入出力topic・型・QoSは従来どおり（/event_camera/tensor、isaac_ros_tensor_msgs/msg/TensorList、GPU FP32 NCHW）。
標準profileは[1,20,120,160]、40ms窓・4ms stride・250Hz。モデル由来YAMLのshape/正規化も保持する。

```bash
ros2 launch openeb_ros2 tensor.launch.py tensor_backend:=cuda_async \
  tensor_config:="$(ros2 pkg prefix openeb_ros2)/share/openeb_ros2/config/tensor_cuda_async_grouped_gpu.yaml"
```

kartのCLI/TUIはkart_bringup READMEを参照。`--evs-aggregation evt3_runs_grouped_gpu`で選択し、
backend未指定時はcuda_asyncを選ぶ。明示backend=cpu/cudaとの併用は起動前に拒否する。
selectorのevent_aggregation既定空文字は従来のTensor YAMLを保持し、CLIの明示値だけchildへoverrideする。

grouped方式は受信callbackから所有権ごとRAWを既存有界packet queueへ渡し、専用workerがscanとCUDA enqueueを行う。
別decode workerや展開済みevent配列/decoded queueは使わず、CUDA stream上のGPU処理は非同期。
1 packetずつ圧縮RAW＋run descriptorをowned pinned/device slotへコピーしてGPU ringへ集計する。
packet上限grouped_max_packet_bytes=1048576、run上限grouped_max_runs=65536を事前確保する。
queue overflow、上限超過、stale破棄、sequence欠落/逆順、geometry変更、decode/CUDA errorでは
parser/GPU ring/scheduleをリセットし、新しいepochで窓を生成できる状態まで再取得してから再開する。
4ms以内のイベント時刻逆転はそのイベントをdropし、streamをresetしない。

evt3_runs_gpu_directは既存のCPU decode/GPU snapshot worker構成を維持。
mutex内でCPU ringをpinnedへ退避してからdirtyをackし、GPU snapshotはlock外で構成する。
転送失敗時は全dirtyを再送し、decoder中のring更新とDMAを競合させない。

診断はevent_aggregation=evt3_runs_grouped_gpu、tensor_upload=compressed_raw_and_runs。
grouped_raw_h2d_bytes/grouped_descriptor_h2d_bytes/grouped_runs/grouped_limit_dropsは診断interval内の値。
groupedのdecode_timeはscan、decode_serviceはGPU submitを含むworker全体で、
decode_scheduling_delayにはCUDA slot再利用待ちも含む。CPU使用率や純粋なscheduler遅延と混同しない。

Linux単独benchmarkのユーザー実行報告でGPU Tensor/CPU参照一致を確認済み。
今回のROS統合はMacでportable/CLI/TUI/launch stub/host C++ stub構文検査まで。
ROS＋SDK＋CUDA build・USB live・DNN併走・Orinの速度は未確認。
独立benchmarkは4ms相当frameでGPUへまとめて送るが、ROS実装はSDK packet単位なので、
同じkernelでも呼出し頻度・queue待ちによって処理時間は変わり得る。
