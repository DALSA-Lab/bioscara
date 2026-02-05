from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():

    scene_dir = PathJoinSubstitution([FindPackageShare('scene_bringup'), 'launch'])
    move_group_dir = PathJoinSubstitution([FindPackageShare('bioscara_arm_gripper_128_moveit_config'), 'launch'])
    rviz_dir = PathJoinSubstitution([FindPackageShare('bioscara_arm_gripper_128_moveit_config'), 'launch'])
    return LaunchDescription(
        [
            DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
            ),
            IncludeLaunchDescription(
                    PathJoinSubstitution([scene_dir, 'bioscara_arm_gripper128.launch.py']),
                    launch_arguments={'use_mock_hardware': LaunchConfiguration("use_mock_hardware")}.items()
            ),
            GroupAction([
                IncludeLaunchDescription(
                PathJoinSubstitution([move_group_dir, 'move_group.launch.py'])
            )
            ],forwarding = False),
            GroupAction([
                IncludeLaunchDescription(
                PathJoinSubstitution([rviz_dir, 'moveit_rviz.launch.py'])
            )
            ],forwarding = False)
        ]
    )