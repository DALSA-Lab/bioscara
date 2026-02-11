# $LICENSE$

from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    control_node = Node(
        package="rqt_controller_manager",
        executable="rqt_controller_manager",
        output="both",
        # prefix=['gdbserver localhost:3000']
        # prefix=['xterm -e gdb -ex run --args']
    )



    return LaunchDescription(
        [
            control_node,
        ]
    )