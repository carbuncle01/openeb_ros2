"""Direct OpenEB input and selectable GPU-backed tensor output; no packet topic input."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = Path(get_package_share_directory("openeb_ros2"))
    selector_path = Path(LaunchConfiguration("pipeline_config").perform(context))
    with selector_path.open(encoding="utf-8") as stream:
        selector = yaml.safe_load(stream)
    backend = LaunchConfiguration("tensor_backend").perform(context)
    if not backend:
        backend = selector["/**/tensor_pipeline"]["ros__parameters"]["tensor_backend"]
    if backend not in ("cpu", "cuda", "cuda_async"):
        raise ValueError("tensor_backend must be cpu, cuda, or cuda_async")
    driver_config = LaunchConfiguration("driver_config").perform(context)
    tensor_config = LaunchConfiguration("tensor_config").perform(context)
    if not tensor_config:
        tensor_config = str(share / "config" / f"tensor_{backend}.yaml")
    image_config = LaunchConfiguration("image_config").perform(context)
    overrides = {"tensor_backend": backend}
    serial = LaunchConfiguration("serial").perform(context)
    if serial:
        overrides["serial"] = serial
    packet = LaunchConfiguration("packet_publish_enabled").perform(context)
    if packet:
        if packet.lower() not in ("true", "false"):
            raise ValueError("packet_publish_enabled must be true or false")
        overrides["packet_publish_enabled"] = packet.lower() == "true"
    return [
        LogInfo(msg=f"OpenEB backend={backend}; configs={driver_config}, {tensor_config}, {image_config}, {selector_path}; overrides={overrides}"),
        Node(package="openeb_ros2", executable="openeb_tensor_pipeline",
             name="tensor_pipeline", namespace=LaunchConfiguration("namespace"),
             parameters=[driver_config, tensor_config, image_config, str(selector_path), overrides],
             output="screen"),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("openeb_ros2"))
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="event_camera"),
        DeclareLaunchArgument("pipeline_config", default_value=str(share / "config/tensor_pipeline.yaml")),
        DeclareLaunchArgument("driver_config", default_value=str(share / "config/driver.yaml")),
        DeclareLaunchArgument("tensor_config", default_value=""),
        DeclareLaunchArgument("image_config", default_value=str(share / "config/visualization.yaml")),
        DeclareLaunchArgument("tensor_backend", default_value=""),
        DeclareLaunchArgument("packet_publish_enabled", default_value=""),
        DeclareLaunchArgument("serial", default_value=""),
        OpaqueFunction(function=_setup),
    ])
