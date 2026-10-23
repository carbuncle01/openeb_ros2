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
