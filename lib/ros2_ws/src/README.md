# Package Architecture
<!-- The following needs a general explaination of packages in ROS2 -->
Source code for a a software module are grouped in packages, for example a hardware interface, custom controller or robot description.

This document aims to illustrate the ROS2 package architecture as it can be found in the [~/bioscara/ROS2/ros2_scara_ws/src](~/bioscara/ROS2/ros2_scara_ws/src) directory.


- TODO: What is a package?

## Remarks on Nomenclature
- The robot arm (without any gripper) is refered to as the "arm" or "bioscara_arm".
- a custom bioscara gripper is refered to as the "gripper" or "bioscara\_gripper\_\<type\>"
- The assembly of an arm and a gripper is a "robot" or "bioscara\_\<arm\>\_\<gripper\>" .
- An environment with a robot and potentially other objects is a "scene".

## Design goals

Modularity and scalability have been a key motivation when designing the package architecture. 

Commercial robots come with their own collection of packages. 

It is expected that new iterations of a gripper are developed. Integration of those must be simplified. TODO: currently not a individual package, only indivdual macros

Modularity achieved two fold:

- standalone packages for arms and grippers.
- Grippers that are simply variants but utilize the same hardware interface, just different hardware description and hardware parameters grouped in a package but with standalone description macros.

Packages and description macros can be swapped via command line arguments or set in scene specific bringup files



The architecture may serve as a blueprint for the future integration of more DALSA robots (custom or commercial) into ROS2. 

TODO: reference to overal modularity description

## Package Types

Throughout the ROS2 workspace one can find different classes of packages whose purposes shall be described in the following.

### Meta Packages

Packages that logically belong together are grouped in meta packages. Meta packages do not contain source code or configuration files but simplify dependency management. For example a hardware meta package may contain the hardware's description, driver, interface, controller and bringup packages. Other packages that depend on that hardware can simply specify the meta package as a dependency and automatically inherit the remaining sub-dependencies. 

The following meta packages are available:

**dalsa_bioscara_arm**: Contains all packages related to the custom bioscara arm hardware component

**dalsa_bioscara_grippers**: Contains all packages related to  the custom bioscara gripper hardware component and its variants. 

**dalsa_controllers**: Contains custom, hardware agnostic controllers.

Hardware component meta packages are structured according to the [RTW Package Structure](https://rtw.b-robotized.com/master/guidelines/robot_package_structure.html) refrence, developed by [dstogl](https://github.com/destogl), a key developer of ros2_control.



<!-- TODO: order of packages. Bringup needs explaination of description first -->

Bringup parses scene description from description package and starts control stack with controller parameters and hardware components



#### Description Packages
A robot description file is an integral part in many ROS2 applications. ROS2 uses an XML based format called Unified Robot Description File (URDF) which is "*a format to describe the kinematics, dynamics, and geometries of robots, independently of software program*" [[1]][urdf], in detail this includes a systems:

- Kinematic description (joints and frame definitions) TODO: Frames and joint graph (vs DH definitions)
- Visual properties for viszualization
- Collision properties defining physical boundaries for trajectory generation
- Mass and Inertial properties for physical simulation (not utilized in this project)
- The urdf can be expanded for specific applications, in this project it addtionally includes:
- ros2_contol configuration
  - interfaces
  - joints
  - hardware interface plugin



[urdf]: https://vbn.aau.dk/ws/files/710175796/main.pdf	"Understanding URDF: A Dataset and Analysis"



Hardware components have a description package that describe the hardwares kinematic chains (joints and frames), collision 
Description packages contains a scene for standalone bringup, and if hardware component a hardware component macro that can be included in other description files to assemble a system. Each component macro loads the components hardware interface and visual, collision and kinematic description. (TODO: explain urdf, xacro, xacro macros)

- Contains a view launch to simply display the geometry visuals and collision bodies, test joint limits using joint state broadcaster and frame transformations.

  <!-- TODO: explain robot assembly -->

Key feature is to dynamicaly assemble description files via xacro to assemble robot condifurations.  

```mermaid
flowchart LR
 subgraph arm["arm"]
        base_link["base_link"]
        link_1["link_1"]
        link_2["link_2"]
        link_3["link_3"]
        link_4["link_3"]
        tool_flange["tool_flange"]
  end
 subgraph gripper["gripper"]
        grp_base_link["grp_base_link"]
        left_jaw_link["left_jaw_link"]
        right_jaw_link["right_jaw_link"]
        tool["tool"]
  end
 subgraph robot["robot"]
        arm
        gripper
  end
 subgraph scene["scene"]
        robot
        world["world"]
  end
    tool_flange -- origin --> grp_base_link
    grp_base_link --> left_jaw_link & right_jaw_link & tool
    base_link -- j2 --> link_1
    link_1 -- j2 --> link_2
    link_2 -- j3 --> link_3
    link_3 -- j4 --> link_4
    link_4 -- flange_joint --> tool_flange
    world -- origin --> base_link

    style arm fill:#FFD600
    style gripper fill:#00C853
    style robot fill:#C8E6C9
    style scene fill:#FFE0B2

```

```mermaid
flowchart LR
 subgraph bioscara["bioscara"]
    direction LR
        arm_urdf_param_junction["arm_urdf_param_junction"]
        arm_urdf_origin>"origin"]
        arm_urdf_parent>"parent"]
        arm_urdf_parameters>"parameters"]
        arm_urdf_prefix>"prefix"]
  end
 subgraph bioscara_arm_ros2_control["bioscara_arm_ros2_control"]
    direction LR
        arm_ctrl_param_junction["arm_ctrl_param_junction"]
        arm_ctrl_use_mock_hardware>"use_mock_hardware"]
        arm_ctrl_parameters>"parameters"]
        arm_ctrl_prefix>"prefix"]
  end
 subgraph load_arm["load_arm"]
    direction TB
        arm_param_junction["arm_param_junction"]
        arm_param_file["**[parameters]** = bioscara_arm_parameters.yaml"]
        bioscara
        bioscara_arm_ros2_control
  end
 subgraph bioscara_gripper_128["bioscara_gripper_128"]
        grp_urdf_p[" "]
  end
 subgraph bioscara_gripper_128_ros2_control["bioscara_gripper_128_ros2_control"]
        grp_ctrl_p[" "]
  end
 subgraph load_gripper["load_gripper"]
        grp_param_junction["grp_param_junction"]
        grp_param_file["**[parameters]** = bioscara_gripper_128_parameters.yaml"]
        bioscara_gripper_128
        bioscara_gripper_128_ros2_control
  end
 subgraph load_robot["load_robot"]
    direction TB
        robot_param_junction["robot_param_junction"]
        grp_origin["**[origin]** = xyz=[0 0 0] rpy=[π 0 -π/2]"]
        grp_parent@{ label: "**[prefix]** = 'tool_flange'" }
        load_arm
        load_gripper
  end
 subgraph scene["scene"]
    direction LR
        use_mock_hardware>"**[use_mock_hardware]**"]
        prefix@{ label: "**[prefix]** = ' '" }
        parent@{ label: "**[prefix]** = 'world'" }
        origin["**[origin]** = xyz=[0 0 0] rpy=[0 0 0]"]
        load_robot
  end
 subgraph bus["symbolic multiple parameters"]
        n1["**[target]** = value "] &  n2[" "] --> legend_junction1 ==> legend_junction2 --> n3["target"]
        legend_junction1
        legend_junction2
  end
 subgraph legend["Legend"]
    direction TB
        defined["defined in macro"]
        arg>"macro argument"]
        bus
  end
    arm_ctrl_param_junction --> arm_ctrl_use_mock_hardware & arm_ctrl_parameters & arm_ctrl_prefix
    arm_urdf_param_junction --> arm_urdf_origin & arm_urdf_parent & arm_urdf_prefix & arm_urdf_parameters
    arm_param_junction ==> arm_ctrl_param_junction & arm_urdf_param_junction
    arm_param_file --> arm_urdf_param_junction & arm_ctrl_param_junction
    grp_param_junction ==> bioscara_gripper_128 & bioscara_gripper_128_ros2_control
    grp_param_file --> bioscara_gripper_128 & bioscara_gripper_128_ros2_control
    grp_origin --> grp_param_junction
    grp_parent --> grp_param_junction
    robot_param_junction ==> grp_param_junction & arm_param_junction
    use_mock_hardware --> robot_param_junction
    origin --> robot_param_junction
    prefix --> robot_param_junction
    parent --> robot_param_junction
    arm_urdf_param_junction@{ shape: sm-circ}
    arm_ctrl_param_junction@{ shape: sm-circ}
    arm_param_junction@{ shape: sm-circ}
    arm_param_file@{ shape: manual-input}
    grp_param_junction@{ shape: sm-circ}
    grp_param_file@{ shape: manual-input}
    robot_param_junction@{ shape: sm-circ}
    grp_origin@{ shape: manual-input}
    grp_parent@{ shape: manual-input}
    prefix@{ shape: manual-input}
    parent@{ shape: manual-input}
    origin@{ shape: manual-input}
    defined@{ shape: manual-input}
    legend_junction1@{ shape: sm-circ}
    legend_junction2@{ shape: sm-circ}

    style load_arm fill:#FFD600
    style load_gripper fill:#00C853
    style load_robot fill:#C8E6C9
    style scene fill:#FFE0B2
```

#### Bringup Packages

- Launch files to start the unit with the entire application stack which can run standalone
  ....

- 

#### Hardware Interface Packages

#### Hardware Driver Packages

Consists of:

**urdf**

**meshes**

**launch**:

config

#### Controllers Packages

- meta package for dalsa_bioscara_arm, the robot arm itself



## Overview
```mermaid
flowchart TB
 subgraph dalsa_bioscara_arm["dalsa_bioscara_arm"]
        bioscara_arm_bringup["bioscara_arm_bringup"]
        bioscara_arm_description["bioscara_arm_description"]
        bioscara_arm_hardware_interface["bioscara_arm_hardware_interface"]
        bioscara_arm_hardware_driver["bioscara_arm_hardware_driver"]
  end
 subgraph dalsa_bioscara_grippers["dalsa_bioscara_grippers"]
        bioscara_gripper_bringup["bioscara_gripper_bringup"]
        bioscara_gripper_descriptions["bioscara_gripper_descriptions"]
        bioscara_gripper_hardware_driver["bioscara_gripper_hardware_driver"]
        bioscara_gripper_hardware_interface["bioscara_gripper_hardware_interface"]
  end
 subgraph dalsa_controllers["dalsa_controllers"]
        single_trigger_controller["single_trigger_controller"]
  end
    scene_description["scene_description"] --> bioscara_arm_description & bioscara_gripper_descriptions
    scene_bringup["scene_bringup"] --> scene_description & dalsa_bioscara_arm & dalsa_bioscara_grippers
    bioscara_arm_bringup --> single_trigger_controller & bioscara_arm_description
    bioscara_arm_description --> bioscara_arm_hardware_interface
    bioscara_arm_hardware_interface --> bioscara_arm_hardware_driver
    bioscara_gripper_bringup --> bioscara_arm_description & bioscara_gripper_descriptions
    bioscara_gripper_hardware_driver --> bioscara_arm_hardware_driver
    bioscara_gripper_descriptions --> bioscara_gripper_hardware_interface
    bioscara_gripper_hardware_interface --> bioscara_gripper_hardware_driver

```

singletirggercontroller as standalone package outside dalsa_biosacra:

is not tied and specific to bioscara



shared namespace between joints and gripper driver for simpler code

same for hardware interface

bioscara_gripper_descriptions i splural since it can host variants of the bioscara gipper. interface stays the same only description and parameters will change






<!-- TODO -->

## Scene Bringup
One-off launch file to launch the specific hardware combination, bioscara_arm and bioscara_gripper_128 in this case. 

would be nice if it could be a single launch file but. Each each setup is slightly different.