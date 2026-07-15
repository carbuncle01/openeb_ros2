# openeb_ros2

ROS 2 Jazzy package containing separate OpenEB driver and preprocessing
components.

## Nodes

### `openeb_driver_node`

Opens an OpenEB camera and publishes compact EVT3 RAW packets as
`event_camera_msgs/msg/EventPacket` on `events_raw`. It does not register a CD
callback and therefore does not request host-side event decoding during normal
operation.

### `openeb_preprocessor_node`

Subscribes to `events_raw` and publishes validated RAW packets on `events`. It
also decodes EVT3 packets with `event_camera_codecs`, accumulates events into
time-slice images, and publishes `sensor_msgs/msg/Image` on `event_image`.
Decoded image processing starts only while an image subscriber exists.

## Build on ROS 2 Jazzy

Place this package in a ROS 2 workspace and build it after sourcing Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select openeb_ros2
source install/setup.bash
```

The package depends on an installed Metavision SDK/OpenEB, plus
`event_camera_msgs`, `event_camera_codecs`, `diagnostic_msgs`, and
`sensor_msgs`. JetPilot installs the CenturyArks SilkyEvCam-enabled OpenEB
build under `/usr/local`.

## Run

Run the driver only:

```bash
ros2 launch openeb_ros2 driver.launch.py
```

Run driver and preprocessor as separate processes:

```bash
ros2 launch openeb_ros2 pipeline.launch.py
```

Run both components in one process with intra-process communication:

```bash
ros2 launch openeb_ros2 composed.launch.py
```

The default topics under namespace `/event_camera` are:

- `/event_camera/events_raw`: RAW EVT3 packets from the driver
- `/event_camera/events`: preprocessor output
- `/event_camera/event_image`: decoded event image (`bgr8` by default)
- `/event_camera/diagnostics`: driver and preprocessor statistics as
  `diagnostic_msgs/msg/DiagnosticArray`

Set a camera serial number when multiple cameras are connected:

```bash
ros2 launch openeb_ros2 composed.launch.py serial:=YOUR_SERIAL
```

Generate images at a selected frequency:

```bash
ros2 launch openeb_ros2 composed.launch.py event_image_fps:=50.0
```

The image output is available from both the standalone and composable
pipelines. `bgr8` renders ON events in blue and OFF events in red; pixels that
receive both polarities within one frame become magenta. A lower-bandwidth
`mono8` output is also available:

```bash
ros2 launch openeb_ros2 composed.launch.py event_image_encoding:=mono8
```

Set `event_image_enabled:=false` to disable the image publisher entirely.
The default `event_image_fps` is 25 Hz. Each image accumulates events decoded
between wall-clock image timer ticks; its actual frequency and timer cost are
reported in `preprocessor_stats`.

## Internal performance statistics

Both nodes collect low-overhead counters internally and publish structured
diagnostics. The default interval is one second and can be changed, or disabled
with zero:

```bash
ros2 launch openeb_ros2 composed.launch.py statistics_interval_s:=1.0
```

The periodic terminal statistics logs are disabled by default. Enable them when
interactive debugging is useful:

```bash
ros2 launch openeb_ros2 composed.launch.py debug:=true
```

For offline debugging, record diagnostics together with the event streams:

```bash
ros2 bag record \
  /event_camera/events_raw \
  /event_camera/events \
  /event_camera/event_image \
  /event_camera/diagnostics
```

The driver reports:

- RAW callback and publish frequency
- input and output bandwidth
- average RAW buffer size
- mean/maximum callback processing time
- callback busy percentage and processing cost normalized per KiB
- mean/maximum RAW callback inter-arrival time
- callbacks discarded when no subscriber exists

The preprocessor reports:

- receive and publish frequency/bandwidth
- average packet size
- mean/maximum callback processing time
- callback busy percentage and processing cost normalized per KiB
- mean/maximum transport latency measured from the driver header stamp
- decoded event rate and mean/maximum decode time
- decode cost normalized per event
- event-image frequency, bandwidth, events per image, and timer cost
- empty, unexpected-encoding, and no-subscriber discard counts

Diagnostic status names end with `driver_stats` and `preprocessor_stats`.
These internal counters should be used for the main performance comparison
because `ros2 topic hz` and `ros2 topic bw` add another subscriber and alter the
load being measured.

## Preprocessor extension point

Extend the RAW packet processing in
`PreprocessorComponent::preprocess()` with a ROS-independent preprocessing
class. The event-image decoder already reports decoded-event count and decode
time through the statistics framework. For additional CPU-heavy processing,
use a bounded queue with explicit overflow handling.
