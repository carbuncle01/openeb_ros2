# ノードparameter一覧

ソースのdeclare_parameterから作成。packet_publish_enabled/raw_recording_dirとpreprocessorのevent_image_window_ms/stride_ms/debug以外は起動時設定。
tensorノードではuse_sim_time以外の実行中変更を拒否する。
config/tensor_*.yamlはdirect pipeline向けのsubscribe_packets=false。単体component既定はtrue。

## driver

| 名前 | 基準値 | 意味 |
|---|---|---|
| `packet_publish_enabled` | `true` | RAW packet topic配信を許可。falseでもdirect tensor生成・RAW記録は継続。動的変更可。 |
| `serial` | `""` | カメラserial。空文字は最初に見つかったカメラ。 |
| `device_format` | `""` | OpenEBの取得format。空文字はカメラ既定。 |
| `bias_file` | `""` | 起動前に適用する.biasの絶対パス。空文字は既定bias。 |
| `encoding` | `"evt3"` | RAW encoding。既定evt3。 |
| `frame_id` | `"event_camera"` | 出力headerのframe。 |
| `raw_recording_enabled` | `false` | native RAW記録・サービス操作を許可。 |
| `raw_recording_auto_start` | `true` | 起動時にRAW保存を開始。 |
| `raw_recording_dir` | `""` | RAW保存先。auto_start有効時は必須。停止中は絶対パスへ動的変更可。 |
| `raw_recording_basename` | `""` | RAWファイル名prefix。 |
| `raw_recording_split_duration_s` | `0.0` | RAW分割周期[s]。0で自動分割なし。 |
| `packet_duration_us` | `1000` | topic専用経路のpacket集約間隔[µs]。directではSDK callbackごとに渡す。 |
| `publish_gap_warning_us` | `4000` | packet処理間隔の警告閾値[µs]。 |
| `packet_size_bytes` | `1000000` | topic専用経路の集約サイズ閾値[bytes]。 |
| `publisher_depth` | `8` | 出力QoS KeepLast深さ。 |
| `statistics_interval_s` | `1.0` | 診断周期[s]。0で無効。 |
| `debug` | `false` | デバッグlogを有効化。 |

## preprocessor

| 名前 | 基準値 | 意味 |
|---|---|---|
| `packet_forward_enabled` | `true` | 検証済みpacketのevents配信。direct可視化では強制false。 |
| `subscribe_packets` | `true` | 単体component時のpacket入力購読。direct executableでは強制false。 |
| `image_queue_capacity` | `4` | direct可視化queueの最大packet数。 |
| `image_queue_max_bytes` | `8388608` | direct可視化queueの最大RAW byte数。 |
| `image_queue_max_age_ms` | `100.0` | 可視化queueの古いpacketを破棄する閾値[ms]。 |
| `expected_encoding` | `"evt3"` | 受け入れるRAW encoding。 |
| `output_frame_id` | `""` | packet出力frame上書き。空は入力を保持。 |
| `drop_empty_packets` | `true` | 空packetを破棄。 |
| `drop_unexpected_encoding` | `true` | encoding不一致を破棄。 |
| `event_image_enabled` | `true` | イベント画像出力を許可。 |
| `event_image_fps` | `25.0` | 旧方式の画像更新頻度[Hz]。 |
| `event_image_window_ms` | `0.0` | 画像の蓄積窓[ms]。0はfpsから導出。 |
| `event_image_stride_ms` | `0.0` | 画像更新間隔[ms]。0はfpsから導出。 |
| `event_image_encoding` | `"bgr8"` | 画像encoding。 |
| `event_image_style` | `"dark"` | dark / gep / red_blue_white。後者は白背景・ON赤・OFF青。 |
| `event_image_percentile` | `90.0` | 描画scaleのpercentile。 |
| `event_image_publish_empty` | `true` | 空画像も出力。 |
| `debug` | `false` | デバッグlogを有効化。 |
| `subscription_depth` | `8` | 入力QoS KeepLast深さ。direct入力では未使用。 |
| `publisher_depth` | `8` | 出力QoS KeepLast深さ。 |
| `event_image_publisher_depth` | `2` | 画像出力QoS深さ。 |
| `statistics_interval_s` | `1.0` | 診断周期[s]。0で無効。 |

## cpu

| 名前 | 基準値 | 意味 |
|---|---|---|
| `bins` | `10` | 時間bin数。 |
| `width` | `212` | 出力tensor幅[pixel]。 |
| `height` | `120` | 出力tensor高さ[pixel]。 |
| `window_ms` | `40.0` | イベント蓄積窓[ms]。 |
| `stride_ms` | `4.0` | 表現の更新間隔[ms]。 |
| `output_rate_hz` | `0.0` | tensor出力周期[Hz]。legacyの0はstrideから導出。 |
| `polarity_mode` | `"separate"` | separateは極性別2B channel、signedは符号付きB channel。 |
| `polarity_layout` | `"polarity_major"` | polarity_majorまたはtime_major。 |
| `temporal_interpolation` | `"none"` | noneはhistogram、linearは時間補間（CPUのみ）。 |
| `incremental_mode` | `"auto"` | auto/off/require。CPUの重複bin再利用。 |
| `representation_backend` | `"cpu"` | cpu/cuda。pipelineのtensor_backendで選択。 |
| `inference_policy` | `"periodic"` | periodicまたはconsumer_driven（従来CUDA）。 |
| `cuda_update_us` | `1000` | 新着eventのGPU転送期限[µs]。 |
| `timestamp_backward_tolerance_us` | `4000` | 破棄する局所時刻逆行の閾値[µs]。超過はstate reset。 |
| `cuda_events_per_transfer` | `8192` | GPU転送batchのevent数。 |
| `inference_watchdog_ms` | `100.0` | consumer_drivenの推論完了待ちwatchdog[ms]。 |
| `tensor_name` | `"input_tensor"` | TensorList.namesに入れるモデル入力名。 |
| `channel_mean` | `[0.0]` | 正規化mean。1要素またはchannel数。 |
| `channel_stddev` | `[1.0]` | 正規化stddev。正値、1要素またはchannel数。 |
| `publish_empty` | `true` | 空windowのtensorも出力。 |
| `use_pinned_host_memory` | `true` | CPU tensor stagingにpinned memoryを使用。 |
| `debug` | `false` | デバッグlogを有効化。 |
| `statistics_interval_s` | `1.0` | 診断周期[s]。0で無効。 |
| `subscription_depth` | `8` | 入力QoS KeepLast深さ。direct入力では未使用。 |
| `publisher_depth` | `4` | 出力QoS KeepLast深さ。 |
| `staging_buffer_count` | `8` | CPU staging bufferの本数。CUDA buffer allocatorのpoolサイズではない。 |
| `diagnostics_topic` | `"tensor_diagnostics"` | 診断出力topic。相対名はnamespace内。 |
| `subscribe_packets` | `false` | 単体component時のpacket入力購読。direct executableでは強制false。 |

## cuda

| 名前 | 基準値 | 意味 |
|---|---|---|
| `bins` | `10` | 時間bin数。 |
| `width` | `212` | 出力tensor幅[pixel]。 |
| `height` | `120` | 出力tensor高さ[pixel]。 |
| `window_ms` | `40.0` | イベント蓄積窓[ms]。 |
| `stride_ms` | `4.0` | 表現の更新間隔[ms]。 |
| `output_rate_hz` | `0.0` | tensor出力周期[Hz]。legacyの0はstrideから導出。 |
| `polarity_mode` | `"separate"` | separateは極性別2B channel、signedは符号付きB channel。 |
| `polarity_layout` | `"polarity_major"` | polarity_majorまたはtime_major。 |
| `temporal_interpolation` | `"none"` | noneはhistogram、linearは時間補間（CPUのみ）。 |
| `incremental_mode` | `"auto"` | auto/off/require。CPUの重複bin再利用。 |
| `representation_backend` | `"cuda"` | cpu/cuda。pipelineのtensor_backendで選択。 |
| `inference_policy` | `"periodic"` | periodicまたはconsumer_driven（従来CUDA）。 |
| `cuda_update_us` | `1000` | 新着eventのGPU転送期限[µs]。 |
| `timestamp_backward_tolerance_us` | `4000` | 破棄する局所時刻逆行の閾値[µs]。超過はstate reset。 |
| `cuda_events_per_transfer` | `8192` | GPU転送batchのevent数。 |
| `inference_watchdog_ms` | `100.0` | consumer_drivenの推論完了待ちwatchdog[ms]。 |
| `tensor_name` | `"input_tensor"` | TensorList.namesに入れるモデル入力名。 |
| `channel_mean` | `[0.0]` | 正規化mean。1要素またはchannel数。 |
| `channel_stddev` | `[1.0]` | 正規化stddev。正値、1要素またはchannel数。 |
| `publish_empty` | `true` | 空windowのtensorも出力。 |
| `use_pinned_host_memory` | `true` | CPU tensor stagingにpinned memoryを使用。 |
| `debug` | `false` | デバッグlogを有効化。 |
| `statistics_interval_s` | `1.0` | 診断周期[s]。0で無効。 |
| `subscription_depth` | `8` | 入力QoS KeepLast深さ。direct入力では未使用。 |
| `publisher_depth` | `4` | 出力QoS KeepLast深さ。 |
| `staging_buffer_count` | `8` | CPU staging bufferの本数。CUDA buffer allocatorのpoolサイズではない。 |
| `diagnostics_topic` | `"tensor_diagnostics"` | 診断出力topic。相対名はnamespace内。 |
| `subscribe_packets` | `false` | 単体component時のpacket入力購読。direct executableでは強制false。 |

## cuda_async

| 名前 | 基準値 | 意味 |
|---|---|---|
| `bins` | `10` | 時間bin数。 |
| `width` | `212` | 出力tensor幅[pixel]。 |
| `height` | `120` | 出力tensor高さ[pixel]。 |
| `window_ms` | `40.0` | イベント蓄積窓[ms]。 |
| `stride_ms` | `4.0` | 表現の更新間隔[ms]。 |
| `output_rate_hz` | `250.0` | tensor出力周期[Hz]。legacyの0はstrideから導出。 |
| `polarity_mode` | `"separate"` | separateは極性別2B channel、signedは符号付きB channel。 |
| `polarity_layout` | `"polarity_major"` | polarity_majorまたはtime_major。 |
| `temporal_interpolation` | `"none"` | noneはhistogram、linearは時間補間（CPUのみ）。 |
| `tensor_name` | `"input_tensor"` | TensorList.namesに入れるモデル入力名。 |
| `channel_mean` | `[0.0]` | 正規化mean。1要素またはchannel数。 |
| `channel_stddev` | `[1.0]` | 正規化stddev。正値、1要素またはchannel数。 |
| `publish_empty` | `true` | 空windowのtensorも出力。 |
| `debug` | `false` | デバッグlogを有効化。 |
| `statistics_interval_s` | `1.0` | 診断周期[s]。0で無効。 |
| `deadline_ms` | `4.0` | async snapshotの期限[ms]。 |
| `timestamp_backward_tolerance_us` | `4000` | 破棄する局所時刻逆行の閾値[µs]。超過はstate reset。 |
| `cuda_events_per_transfer` | `8192` | GPU転送batchのevent数。 |
| `gpu_chunk_events` | `8192` | GPU workerの処理chunk数。chunk間でsnapshot期限を確認。 |
| `packet_queue_capacity` | `64` | async RAW入力queueの最大packet数。 |
| `decoded_queue_capacity` | `64` | async decode済みqueueの最大batch数。 |
| `decode_buffer_pool_capacity` | `4` | decode vector再利用poolの本数。 |
| `decode_buffer_pool_max_events` | `524288` | poolへ保持するvector最大event数。 |
| `max_queue_age_ms` | `20.0` | 古いqueued batchの破棄閾値[ms]。 |
| `subscription_depth` | `16` | 入力QoS KeepLast深さ。direct入力では未使用。 |
| `publisher_depth` | `8` | 出力QoS KeepLast深さ。 |
| `diagnostics_topic` | `"tensor_diagnostics"` | 診断出力topic。相対名はnamespace内。 |
| `subscribe_packets` | `false` | 単体component時のpacket入力購読。direct executableでは強制false。 |


## 連携・可視化profile

config/driver_bag_linked.yamlはdriver.yamlと同じ全parameterを持ち、RAW有効・auto_start=false・
split_duration=0・保存先空・packet配信OFFを設定する。保存先はbag managerが停止中に設定。
config/visualization.yamlはpreprocessor.yamlと同じ全parameterを持ち、subscribe_packets=false・
packet_forward_enabled=false・style=red_blue_white・window/stride=40 msを設定する。
単体preprocessor既定はdark/topic入力、integrated pipelineではこの可視化profileを使う。
