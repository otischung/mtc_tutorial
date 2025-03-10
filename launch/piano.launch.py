# This is the launch file for mtc.cpp node
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_v4", package_name="robot_moveit_config").to_dict()

    # MTC Demo node
    piano = Node(
        package="mtc_tutorial",
        executable="mtc",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([piano])
