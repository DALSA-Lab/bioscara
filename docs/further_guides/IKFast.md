# Creating a IKFast Inverse Kinematics Plugin
:::{important}
This guide can only serve as a starting point since the desired result could not be achieved!
:::

The IKFast is an analytical inverse kinematics solver that automatically generates a robots IK solver code based on its URDF file. This guide documents which steps have been undertaken to generate the IKFast solver plugin for MoveIt. 

The result however has always been that the robot did not move when dragging on its handles in MoveIt.

## Background
Please refer to the original guide on the MoveIt documentation page [here](https://moveit.picknik.ai/main/doc/examples/ikfast/ikfast_tutorial.html) for the information about the individual commands.

## Plugin Creation

make sure Docker is installed: 
```bash
docker -v
```

If that is not the case install Docker by following the guide [here](https://docs.docker.com/engine/install/ubuntu/).

if that is done, make sure your user is added to the "docker" group:
```bash
sudo usermod -aG docker ${USER}
```

log out and back in to apply the new group:
```bash
su -l ${USER}
```

navigate to the *lib/ros2_ws/src/dalsa_bioscara_arm* directory:
```bash
cd lib/ros2_ws/src/dalsa_bioscara_arm
```

export the arm as an ENV variable:
```bash
export MYROBOT_NAME="bioscara_arm"
```

Create a urdf file from the *scene.xacro* which will be parsed by the plugin generation script.
```bash
ros2 run xacro xacro -o $MYROBOT_NAME.urdf bioscara_arm_description/urdf/scene.xacro
```

Then create then call the plugin creation script:
```bash
ros2 run moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype <type> $MYROBOT_NAME.urdf arm base_link tool_flange
```
On first run this will take a while to pull and build the docker images.

Select one 4-DOF \<type\> from the following ([source](https://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types)):

- **Transform6D** - end effector reaches desired 6D transformation
- **Rotation3D** - end effector reaches desired 3D rotation
- **Translation3D** - end effector origin reaches desired 3D translation
- **Direction3D** - direction on end effector coordinate system reaches desired direction
- **Ray4D** - ray on end effector coordinate system reaches desired global ray
- **Lookat3D** - direction on end effector coordinate system points to desired 3D position
- **TranslationDirection5D** - end effector origin and direction reaches desired 3D translation and direction. Can be thought of as Ray IK where the origin of the ray must coincide.
- **TranslationXY2D** - end effector origin reaches desired XY translation position, Z is ignored. The coordinate system with relative to the base link.
- **TranslationLocalGlobal6D** - local point on end effector origin reaches desired 3D global point. Because both local point and global point can be specified, there are 6 values.
- **TranslationXAxisAngle4D**, **TranslationYAxisAngle4D**, **TranslationZAxisAngle4D** - end effector origin reaches desired 3D translation, manipulator direction makes a specific angle with x/y/z-axis (defined in the manipulator base link’s coordinate system)
- **TranslationXAxisAngleZNorm4D**, **TranslationYAxisAngleXNorm4D**, **TranslationZAxisAngleYNorm4D** - end effector origin reaches desired 3D translation, manipulator direction needs to be orthogonal to z, x, or y axis and be rotated at a certain angle starting from the x, y, or z axis (defined in the manipulator base link’s coordinate system)

None of the last 6 types have yielded a successfull result yet.

## Build the new Plugin Package
Navigate to the root of the workspace:

```bash
cd lib/ros2_ws
```

and build the new package:
```bash
colcon build --symlink-install --mixin release --packages-select bioscara_arm_ikfast_plugin
```

## Change the kinematics.yaml
Almost done, change the *lib/ros2_ws/src/moveit_configurations/bioscara_arm_gripper_128_moveit_config/config/kinematics.yaml* file to the new IK solver:
```yaml
arm:
  kinematics_solver: bioscara_arm/IKFastKinematicsPlugin
  ...
```

## Try it out
Start the robot (mock or hardware), move_group and rviz as described in the [guide](../getting_started/operate.md).