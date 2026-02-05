# $LICENSE$

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    LogInfo
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    

    # Initialize Arguments

    # Use the arms controller manager parameters (for now)
    controller_manager_package = "bioscara_arm_bringup"
    controller_manager_file = "bioscara_controller_manager.yaml"
    arm_controller_package = "bioscara_arm_bringup"
    arm_controller_file = "bioscara_arm_controllers.yaml"
    gripper_controller_package = "bioscara_gripper_bringup"
    gripper_controller_file = "bioscara_gripper_controllers.yaml"
    scene_description_package = "scene_descriptions"
    scene_macro = "scene.xacro"
    arm_description_package = "bioscara_arm_description"
    arm_macro = "bioscara_arm.xacro"
    gripper_description_package = "bioscara_gripper_descriptions"
    gripper_macro = "bioscara_gripper_128.xacro"
    prefix = ""
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    gui = LaunchConfiguration("gui")


    # assemble robot via XACRO
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(scene_description_package), "urdf", scene_macro]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "arm_description_package:=",
            arm_description_package,
            " ",
            "arm_macro:=",
            arm_macro,
            " ",
            "gripper_description_package:=",
            gripper_description_package,
            " ",
            "gripper_macro:=",
            gripper_macro,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    arm_controller_file = PathJoinSubstitution(
        [FindPackageShare(arm_controller_package), "config", arm_controller_file]
    )

    gripper_controller_file = PathJoinSubstitution(
        [FindPackageShare(gripper_controller_package), "config", gripper_controller_file]
    )
    controller_manager_file = PathJoinSubstitution(
        [FindPackageShare(controller_manager_package), "config", controller_manager_file]
    )

    # use scene description rviz config file (for now)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(scene_description_package), "rviz", "display.rviz"]
    )

    # Start the ros2_control controller manager node that loads the controller(s) (JTC in my case)
    # and on their request request resources from the resource manager to interact with the hardware
    # The controller manager receives the robot description and controler configuration file as parameters.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, 
                    controller_manager_file, 
                    arm_controller_file, 
                    gripper_controller_file],
        # prefix=['gdbserver localhost:3000']
    )

    # start the robot state publisher node which gets the robot description file as paramter
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    rqt_joint_trajectory_controller_node = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        output="log",
        condition=IfCondition(gui),
    )

    # uses the controller manager to spawn joint state broadcaster.
    # The joint_state_broadcaster is not actually a controller but is treated as such.
    # it publishes/broadcasts the joint states. 
    # Spawning simply starts the controller and puts it into active state.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # spawn the controller manager using the controller manager spawner.
    robot_controllers = ["velocity_joint_trajectory_controller",
                        "homing_controller", 
                        "gripper_controller"]

    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager","--inactive"],
            )
        ]

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = (
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[
                    TimerAction(
                        period=1.0,
                        actions=[joint_state_broadcaster_spawner],
                    ),
                ],
            )
        )
    )

    # Delay rviz start after Joint State Broadcaster to avoid unnecessary warning output.
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
            rviz_node,
            rqt_joint_trajectory_controller_node,
            ],
        )
    )

    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[
                        TimerAction(
                            period=3.0,
                            actions=[controller],
                        ),
                    ],
                )
            )
        ]

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            LogInfo(msg=['Using mock hardware: ', use_mock_hardware])
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )