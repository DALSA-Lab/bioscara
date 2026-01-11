# Robot Gripper  
The current gripper is driven by a PWM controlled RC servo. The frequency is 50 Hz. The PWM should be a hardware generated PWM as software PWM has potentially high jitter which could cause overheating. The gripper is controlled through its C++ API defined in the *bioscara_gripper_hardware_driver* package. <!-- TODO: link to doxygen -->

## Usage
This system test package contains a simple programm that allows to manually control the gripper actuator. The program can be used in two ways:
1) Setting the desired width. This can only be used when the correct reduction and offset is known.
If they are unkown use the 2) mode and follow the steps described there.
2) Setting the servo angle directly. This should be used for testing, positioning when mounting and dismounting and to calculate reduction and offset (explained [here](#calculating-reduction-and-offset)).

First build the package with Colcon, then you can run it after it is sourced. 
### Building
Follow the steps described TODO <!-- TODO: link to build instructions --> to build the package and all its dependencies.

```bash
cd ~/bioscara/ROS2/ros2_scara_ws
colcon build --symlink-install --packages-select gripper_manual_control bioscara_gripper_hardware_driver
```
### Running
In a new terminal:
```bash
cd ~/bioscara/ROS2/ros2_scara_ws
source install/local_setup.sh
ros2 run gripper_example manual_control 
 ```

Sourcing the executables is only is necessary once after opening a new terminal.

After running follow the instructions in the script.

<!-- TODO: Link to BaseGripper Constructor, same stuff documented -->
## Calculating reduction and offset
The gripper has the reduction $r$ and offset $o$ parameters which are used to translate from a desired gripper width to the servo angle. The relationship between gripper width $w$ and acutator angle $\alpha$ is as follows:
$$
\alpha = r (w-o)
$$

To determine these parameters execute the following steps:

1. Manually set the gripper to an open position by setting a actuator angle. Be carefull to not exceed the physical limits of the gripper since the actuator is strong enough to break PLA before stalling.
2. Measure the gripper width $w_1$ and note the set actuator angle $\alpha_1$.
3. Move the gripper to a more closed position that still allows you to accurately measure the width
4. Measure the second width $w_2$ and note the corresponding angle $\alpha_2$
5. Calculate the offset $o$:
$$
o = \frac{\alpha_1 w_2 -  \alpha_2 w_1}{\alpha_1 - \alpha_2}
$$
6. Calculate the reduction $r$:
$$
r = \frac{\alpha_1}{w_1 - o}
$$