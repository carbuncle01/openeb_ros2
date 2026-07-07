from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    serial = LaunchConfiguration("serial")
    frame_id = LaunchConfiguration("frame_id")
    packet_duration_us = ParameterValue(
        LaunchConfiguration("packet_duration_us"), value_type=int
    )
    statistics_interval_s = ParameterValue(
        LaunchConfiguration("statistics_interval_s"), value_type=float
    )
    debug = ParameterValue(LaunchConfiguration("debug"), value_type=bool)
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
                        "frame_id": frame_id,
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
            DeclareLaunchArgument("frame_id", default_value="event_camera"),
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
