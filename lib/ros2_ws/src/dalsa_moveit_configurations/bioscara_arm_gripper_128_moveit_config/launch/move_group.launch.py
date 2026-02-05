from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch_ros.actions import Node
from launch import LaunchDescription



def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("scene", package_name="bioscara_arm_gripper_128_moveit_config")
                    .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
                    .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities
            # {"publish_planning_scene": True},
            # {"publish_geometry_updates": True},
            # {"publish_state_updates": True},
            # {"publish_transforms_updates": True},
        ],
        # prefix=['gdbserver localhost:3000']
    )

    return LaunchDescription(
        [
            run_move_group_node,
        ]
    )

    # return generate_move_group_launch(moveit_config)
