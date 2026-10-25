#!/usr/bin/env python3
"""Check declared parameter coverage and mode-specific YAML without ROS dependencies."""
import ast
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def parameters(path):
    return set(re.findall(r'declare_parameter(?:<[^;]*?>)?\(\s*"([\w]+)"\s*,', path.read_text()))


def configured(path):
    result = {}
    for line in path.read_text().splitlines():
        if line.startswith("    ") and ": " in line:
            name, value = line.strip().split(": ", 1)
            result[name] = json.loads(value)
    return result


def main():
    pairs = [("driver_component.cpp", "driver.yaml"),
             ("preprocessor_component.cpp", "preprocessor.yaml"),
             ("event_tensor_encoder_node.cpp", "tensor_cpu.yaml"),
             ("event_tensor_encoder_node.cpp", "tensor_cuda.yaml"),
             ("async_event_tensor_preprocessor_node.cpp", "tensor_cuda_async.yaml"),
             ("async_event_tensor_preprocessor_node.cpp", "tensor_cuda_async_runs.yaml"),
             ("async_event_tensor_preprocessor_node.cpp", "tensor_cuda_async_runs_gpu.yaml")]
    for source, config in pairs:
        declared = parameters(ROOT / "src" / source)
        actual = configured(ROOT / "config" / config)
        assert declared == actual.keys(), (source, declared - actual.keys(), actual.keys() - declared)
        if config.startswith("tensor_"):
            assert not actual["subscribe_packets"]
            assert actual["bins"] == 10 and actual["polarity_mode"] == "separate"
            assert actual["width"] == 160 and actual["height"] == 120
            assert re.search(r'declare_parameter<std::int64_t>\("width", 160\)',
                             (ROOT / "src" / source).read_text())
    for config in ("driver_bag_linked.yaml",):
        assert parameters(ROOT / "src/driver_component.cpp") == configured(ROOT / "config" / config).keys()
        linked = configured(ROOT / "config" / config)
        assert linked["raw_recording_enabled"] and not linked["raw_recording_auto_start"]
        assert linked["raw_recording_split_duration_s"] == 0.0
    visual = configured(ROOT / "config/visualization.yaml")
    assert parameters(ROOT / "src/preprocessor_component.cpp") == visual.keys()
    assert visual["event_image_style"] == "red_blue_white"
    assert not visual["subscribe_packets"] and not visual["packet_forward_enabled"]
    assert visual["event_image_encoding"] == "bgr8"
    assert configured(ROOT / "config/tensor_cpu.yaml")["representation_backend"] == "cpu"
    assert configured(ROOT / "config/tensor_cuda.yaml")["representation_backend"] == "cuda"
    for path in (ROOT / "launch").glob("*.py"):
        ast.parse(path.read_text(), filename=str(path))
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {e.text for e in package.findall("depend")}
    assert "jetpilot_msgs" not in dependencies
    assert {"cuda_buffer", "tensor_msgs", "isaac_ros_tensor_msgs"} <= dependencies
    assert package.find("buildtool_depend").text == "ament_cmake_auto"
    print("PASS: 7 configurations cover declared parameters; backend contracts, launch syntax, package dependencies")


if __name__ == "__main__":
    main()
