import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "exe",
            description="Program to execute",
        )
    )
    logger = LaunchConfiguration("log_level")
    exe = LaunchConfiguration("exe")

    moveit_config = (
        MoveItConfigsBuilder("scene", package_name="bioscara_arm_gripper_128_moveit_config")
        # .planning_pipelines(pipelines=["ompl"])
        # .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    package = "dalsa_motion_plans"
    node = Node(
        package=package,
        executable=exe,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
        arguments=['--ros-args', '--log-level', logger]
    )

    return LaunchDescription(declared_arguments
                             + [node])
