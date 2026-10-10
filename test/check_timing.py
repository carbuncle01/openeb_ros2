"""Portable timing serialization check with ROS stubs, not ROS integration."""
import json
from pathlib import Path
import subprocess
import tempfile
STUBS = {'rclcpp/rclcpp.hpp': '#pragma once\n#include <memory>\n#include <string>\n#include <cstdint>\nnamespace rclcpp {\ninline std::string captured;\nstruct QoS {explicit QoS(int) {} QoS &reliable(){return *this;}};\ntemplate<class T> struct Publisher {using SharedPtr=std::shared_ptr<Publisher<T>>;int get_subscription_count(){return 1;}void publish(const T&m){captured=m.data;}};\nstruct Node {template<class T>auto create_publisher(const char*,QoS){return std::make_shared<Publisher<T>>();}struct Time {std::int64_t nanoseconds(){return 9000001234LL;}};Time now(){return {};}};\n}\n', 'std_msgs/msg/header.hpp': '#pragma once\n#include <string>\n#include <cstdint>\nnamespace std_msgs {namespace msg {struct Header {struct Stamp {std::int32_t sec=0;std::uint32_t nanosec=0;} stamp; std::string frame_id;};}}\n', 'std_msgs/msg/string.hpp': '#pragma once\n#include <string>\nnamespace std_msgs {namespace msg {struct String {std::string data;};}}\n'}
SOURCE = '#include "openeb_ros2/tensor_timing.hpp"\n#include <iostream>\nint main(){rclcpp::Node node;openeb_ros2::TensorTiming timing(node);std_msgs::msg::Header h;h.stamp.sec=8;h.frame_id="camera\\"test";timing.publish(h,1000,41000,8000000000LL,40000,"tensor",20,120,212);std::cout<<rclcpp::captured<<\'\\n\';timing.reset_epoch();timing.publish(h,2000,42000,8000001000LL,41000,"tensor",20,120,212);std::cout<<rclcpp::captured<<\'\\n\';}\n'
with tempfile.TemporaryDirectory() as folder:
    root = Path(folder)
    for name, content in STUBS.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source = root / "test.cpp"
    source.write_text(SOURCE)
    binary = root / "test"
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I"+str(root), "-I"+str(Path(__file__).resolve().parents[1]/"include"),
                    str(source), "-o", str(binary)], check=True)
    a,b = [json.loads(x) for x in subprocess.check_output([str(binary)], text=True).splitlines()]
    assert a["window_start_ros_ns"] == 8001000000
    assert a["window_end_ros_ns"] == 8041000000
    assert a["tensor_stamp_ros_ns"] == 8000000000
    assert a["frame_id"] == 'camera"test'
    assert (a["sequence"], b["sequence"], b["sensor_epoch"]) == (0,1,1)
    assert (100+900)*1000+a["sensor_to_ros_offset_ns"] == a["window_start_ros_ns"]
print("PASS timing serialization/mapping/epoch (ROS stubs only)")
