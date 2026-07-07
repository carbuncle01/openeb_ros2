from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="event_camera"),
            DeclareLaunchArgument("serial", default_value=""),
            DeclareLaunchArgument("frame_id", default_value="event_camera"),
            DeclareLaunchArgument("packet_duration_us", default_value="1000"),
            DeclareLaunchArgument("statistics_interval_s", default_value="1.0"),
            Node(
                package="openeb_ros2",
                executable="openeb_driver_node",
                namespace=namespace,
                name="event_camera_driver",
                output="screen",
                parameters=[
                    {
                        "serial": LaunchConfiguration("serial"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "packet_duration_us": ParameterValue(
                            LaunchConfiguration("packet_duration_us"),
                            value_type=int,
                        ),
                        "statistics_interval_s": ParameterValue(
                            LaunchConfiguration("statistics_interval_s"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
