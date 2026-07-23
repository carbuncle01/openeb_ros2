from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
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
            Node(
                package="openeb_ros2",
                executable="openeb_driver_node",
                namespace=namespace,
                name="event_camera_driver",
                output="screen",
                parameters=[
                    {
                        "serial": LaunchConfiguration("serial"),
                        "device_format": LaunchConfiguration("device_format"),
                        "frame_id": LaunchConfiguration("frame_id"),
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
                        "packet_duration_us": ParameterValue(
                            LaunchConfiguration("packet_duration_us"),
                            value_type=int,
                        ),
                        "statistics_interval_s": ParameterValue(
                            LaunchConfiguration("statistics_interval_s"),
                            value_type=float,
                        ),
                        "debug": debug,
                    }
                ],
            ),
        ]
    )
