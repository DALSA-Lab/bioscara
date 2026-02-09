# Standalone Gripper Operation
This document describes the simplest way to manually control the gripper. This is desireable for development and necessary after assembly to recalibrate the grippers reduction and offset. The current gripper is driven by a PWM controlled RC servo. The frequency is 50 Hz. The PWM should be a hardware generated PWM as software PWM has potentially high jitter which could cause overheating. The gripper is controlled through its C++ API defined in the *bioscara_gripper_hardware_driver::Gripper* object.


## Connect to the Robot Controller
Follow the [networking tutorial](../getting_started/network.md) to establish a connection to the robot controller. When a connection has been established login to the controller via ssh:
```bash
ssh scara@<ip-adress>
```
Type the password **dtubio** when prompted and hit enter. You should now have a terminal session on the robot controller.


## Manual Operation
The gripper can be manually controller by executing the `gripper_manual_control` program. First, ensure that the entire workspace is built by following the [build](../getting_started/building.md) instructions. At a minimum, the `gripper_manual_control` and `bioscara_gripper_hardware_driver` packages must be built. You can manually build them by executing:
```bash
cd lib/ros2_ws/
colcon build --symlink-install --packages-select gripper_manual_control bioscara_gripper_hardware_driver
```

This system test package contains a simple programm that allows to manually control the gripper actuator. The program can be used in two ways:
1) Setting the desired width. This can only be used when the correct reduction and offset is known. **If they are unkown**, use the 2) mode and follow the steps described there.
2) Setting the servo angle directly. This should be used for testing, positioning when mounting and dismounting and to calculate reduction and offset (explained [here](#calculating-reduction-and-offset)).


### Running the program
Open a new terminal session on the robot controller, navigate to the *lib/ros2_ws/* directory and source it.
```bash
cd lib/ros2_ws/
source install/local_setup.sh
```
:::{tip}
Sourcing the executables is only is necessary once after opening a new terminal.
:::

You can then run the executable:
```bash
ros2 run gripper_manual_control manual_control
```
Now follow the instructions given in the console output.

## Calculating reduction and offset
The gripper has the reduction $r$ and offset $o$ parameters which are used to translate from a desired gripper width to the servo angle. These paramters must be supplied to the gripper hardware interface when used for normal operation. 

:::{warning}
The gripper will be damaged, if the reduction and offset paramters are incorrect. The parameters must always be calibrated after dissassembly and on initial commissioning.
:::

The relationship between gripper width $w$ and acutator angle $\alpha$ is as follows:
$$
\alpha = r (w-o)
$$

To determine these parameters execute the following steps:

1. Manually set the gripper to an open position by setting an actuator angle. Be careful to not exceed the physical limits of the gripper since the actuator is strong enough to break PLA before stalling.
2. Measure the gripper width between the jaws $w_1$ and note the commanded actuator angle $\alpha_1$.
3. Move the gripper to a more closed position that still allows you to accurately measure the width.
4. Measure the second width $w_2$ and note the corresponding angle $\alpha_2$.
5. Calculate the offset $o$:

$$
o = \frac{\alpha_1 w_2 -  \alpha_2 w_1}{\alpha_1 - \alpha_2}
$$

6. Calculate the reduction $r$:

$$
r = \frac{\alpha_1}{w_1 - o}
$$