from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    serial = LaunchConfiguration("serial")
    device_format = LaunchConfiguration("device_format")
    frame_id = LaunchConfiguration("frame_id")
    packet_duration_us = ParameterValue(
        LaunchConfiguration("packet_duration_us"), value_type=int
    )
    statistics_interval_s = ParameterValue(
        LaunchConfiguration("statistics_interval_s"), value_type=float
    )
    debug = ParameterValue(LaunchConfiguration("debug"), value_type=bool)
    raw_recording_enabled = ParameterValue(
        LaunchConfiguration("raw_recording_enabled"), value_type=bool
    )
    raw_recording_auto_start = ParameterValue(
        LaunchConfiguration("raw_recording_auto_start"), value_type=bool
    )
    raw_recording_split_duration_s = ParameterValue(
        LaunchConfiguration("raw_recording_split_duration_s"), value_type=float
    )
    event_image_enabled = ParameterValue(
        LaunchConfiguration("event_image_enabled"), value_type=bool
    )
    event_image_fps = ParameterValue(
        LaunchConfiguration("event_image_fps"), value_type=float
    )
    event_image_publish_empty = ParameterValue(
        LaunchConfiguration("event_image_publish_empty"), value_type=bool
    )
    event_image_publisher_depth = ParameterValue(
        LaunchConfiguration("event_image_publisher_depth"), value_type=int
    )

    container = ComposableNodeContainer(
        name="openeb_pipeline_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="openeb_ros2",
                plugin="openeb_ros2::DriverComponent",
                namespace=namespace,
                name="event_camera_driver",
                parameters=[
                    {
                        "serial": serial,
                        "device_format": device_format,
                        "frame_id": frame_id,
                        "raw_recording_enabled": raw_recording_enabled,
                        "raw_recording_request_topic": LaunchConfiguration(
                            "raw_recording_request_topic"
                        ),
                        "raw_recording_auto_start": raw_recording_auto_start,
                        "raw_recording_dir": LaunchConfiguration("raw_recording_dir"),
                        "raw_recording_basename": LaunchConfiguration(
                            "raw_recording_basename"
                        ),
                        "raw_recording_split_duration_s": raw_recording_split_duration_s,
                        "packet_duration_us": packet_duration_us,
                        "statistics_interval_s": statistics_interval_s,
                        "debug": debug,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="openeb_ros2",
                plugin="openeb_ros2::PreprocessorComponent",
                namespace=namespace,
                name="event_preprocessor",
                parameters=[
                    {
                        "statistics_interval_s": statistics_interval_s,
                        "debug": debug,
                        "event_image_enabled": event_image_enabled,
                        "event_image_fps": event_image_fps,
                        "event_image_encoding": LaunchConfiguration(
                            "event_image_encoding"
                        ),
                        "event_image_publish_empty": event_image_publish_empty,
                        "event_image_publisher_depth": event_image_publisher_depth,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="event_camera"),
            DeclareLaunchArgument("serial", default_value=""),
            DeclareLaunchArgument("device_format", default_value=""),
            DeclareLaunchArgument("frame_id", default_value="event_camera"),
            DeclareLaunchArgument("raw_recording_enabled", default_value="false"),
            DeclareLaunchArgument(
                "raw_recording_request_topic", default_value="raw_recording/request"
            ),
            DeclareLaunchArgument("raw_recording_auto_start", default_value="true"),
            DeclareLaunchArgument("raw_recording_dir", default_value=""),
            DeclareLaunchArgument("raw_recording_basename", default_value="openeb"),
            DeclareLaunchArgument(
                "raw_recording_split_duration_s", default_value="0.0"
            ),
            DeclareLaunchArgument("packet_duration_us", default_value="1000"),
            DeclareLaunchArgument("statistics_interval_s", default_value="1.0"),
            DeclareLaunchArgument("debug", default_value="false"),
            DeclareLaunchArgument("event_image_enabled", default_value="true"),
            DeclareLaunchArgument("event_image_fps", default_value="25.0"),
            DeclareLaunchArgument("event_image_encoding", default_value="bgr8"),
            DeclareLaunchArgument(
                "event_image_publish_empty", default_value="true"
            ),
            DeclareLaunchArgument(
                "event_image_publisher_depth", default_value="2"
            ),
            container,
        ]
    )
