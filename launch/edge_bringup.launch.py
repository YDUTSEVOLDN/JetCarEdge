from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    car_id = LaunchConfiguration("car_id")
    stream_id = LaunchConfiguration("stream_id")
    cloud_url = LaunchConfiguration("cloud_url")
    camera_topic = LaunchConfiguration("camera_topic")
    start_base = LaunchConfiguration("start_base")
    start_camera = LaunchConfiguration("start_camera")
    frame_server_port = LaunchConfiguration("frame_server_port")

    base_driver = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "yahboomcar_bringup",
            "Mcnamu_driver_X3",
        ],
        condition=IfCondition(start_base),
        output="screen",
    )

    camera_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "astra_camera",
            "astro_pro_plus.launch.xml",
            "enable_color:=true",
            "enable_depth:=false",
        ],
        condition=IfCondition(start_camera),
        output="screen",
    )

    edge_node = Node(
        package="jetcar_edge",
        executable="edge_upload_node",
        name="jetcar_edge_upload",
        output="screen",
        parameters=[
            {
                "car_id": car_id,
                "stream_id": stream_id,
                "cloud_url": cloud_url,
                "camera_topic": camera_topic,
                "algorithm_ids": "",
                "frame_server_port": frame_server_port,
                "docker_orchestrator_enabled": False,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("car_id", default_value="car_001"),
            DeclareLaunchArgument("stream_id", default_value="camera_front"),
            DeclareLaunchArgument(
                "cloud_url",
                default_value="ws://192.168.175.90:8000/ws/video/car_001/camera_front/edge",
            ),
            DeclareLaunchArgument("camera_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("frame_server_port", default_value="8100"),
            base_driver,
            camera_launch,
            edge_node,
        ]
    )
