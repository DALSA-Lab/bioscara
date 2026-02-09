# ROS2 Package Architecture
The *lib/ros2_ws/src* directory in the ROS2 workspace contains all packages needed to develop, test and operate the Bioscara robot.
Logically coherent software is combined in a package which is useful for distribution and dependecy management.

## Remarks on Nomenclature
- The robot arm (without any gripper) is refered to as the "arm" or "bioscara_arm".
- a custom bioscara gripper is refered to as the "gripper" or "bioscara\_gripper\_\<type\>"
- The assembly of an arm and a gripper is a "robot" or "bioscara\_\<arm\>\_\<gripper\>" .
- An environment with a robot and potentially other objects is a "scene".

## Design goals
The package architecture is designed on modularity, since it is expected future variants of hard- and software will be developed. The modular approach allows to mix and swap old the software. Modularity achieved two fold:

- Standalone packages for arms and grippers.
- Grippers that are simply variants but utilize the same hardware interface, just different hardware description and hardware parameters grouped in a package but with standalone description macros.
- Packages and description macros can be swapped via command line arguments or set in scene specific bringup files.

The architecture may serve as a blueprint for the future integration of more DALSA robots (custom or commercial) into ROS2. 

## Additional MoveIt2 packages
Unfortunately it was necessary to build MoveIt2 from source to use features from the MoveIt Task Constructor, installing MoveIt from source adds many packages to the workspace.
To build the workspace follow the [compilation instructions](building.md).


## Meta Packages
Packages that logically belong together are grouped in meta packages. Meta packages do not contain source code or configuration files but simplify dependency management. For example a hardware meta package may contain the hardware's description, driver, interface, controller and bringup packages. Other packages that depend on that hardware can simply specify the meta package as a dependency and automatically inherit the remaining sub-dependencies. 

The following meta packages are available:

**dalsa_bioscara_arm**: Contains all packages related to the custom bioscara arm hardware component

**dalsa_bioscara_grippers**: Contains all packages related to  the custom bioscara gripper hardware component and its variants. 

**dalsa_controllers**: Contains custom, hardware agnostic controllers.

**dalsa_moveit_configurations**: Contains MoveIt2 configurations.

**system_test_packages**: Contain different simple programs to execute system tests.


## Overview
The diagram below shows a simple package architecture and dependency diagram. It does not yet include the MoveIt configuration package. The individual packages are grouped by their meta package.

```{mermaid}
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
  subgraph dalsa_moveit_configurations["dalsa_moveit_configurations"]
        bioscara_arm_gripper_128_moveit_config["bioscara_arm_gripper_128_moveit_config"]
  end
    
    scene_descriptions["scene_descriptions"] --> bioscara_arm_description & bioscara_gripper_descriptions
    scene_bringup["scene_bringup"] --> scene_descriptions & dalsa_bioscara_arm & dalsa_bioscara_grippers & bioscara_rviz_plugin
    bioscara_arm_bringup --> single_trigger_controller & bioscara_arm_description
    bioscara_arm_description --> bioscara_arm_hardware_interface
    bioscara_arm_hardware_interface --> bioscara_arm_hardware_driver
    bioscara_gripper_bringup --> bioscara_gripper_descriptions
    bioscara_gripper_hardware_driver --> bioscara_arm_hardware_driver
    bioscara_gripper_descriptions --> bioscara_gripper_hardware_interface
    bioscara_gripper_hardware_interface --> bioscara_gripper_hardware_driver
    bioscara_rviz_plugin["bioscara_rviz_plugin"] --> dalsa_bioscara_arm & dalsa_bioscara_grippers
    dalsa_motion_plans["dalsa_motion_plans"] --> bioscara_arm_gripper_128_moveit_config 
    bioscara_arm_gripper_128_moveit_config --> scene_bringup
```


## Hardware component package
Hardware component meta packages are structured according to the [RTW Package Structure](https://rtw.b-robotized.com/master/guidelines/robot_package_structure.html) refrence, developed by [dstogl](https://github.com/destogl), a key developer of ros2_control. Each hardware component *dalsa_bioscara_grippers* and *dalsa_bioscara_arm*, i.e. the arm and the gripper have meta package structured as follows, using gripper as an example:
```
dalsa_bioscara_grippers/
├── bioscara_gripper_bringup                                # The bringup package is used to launch a hardware component, either standalone or from another bringup package
│   ├── CMakeLists.txt                                      
│   ├── config                                                                                       
│   │   ├── bioscara_gripper_controller_manager.yaml        # ros2_control controller manager paramters if used stand-alone
│   │   └── bioscara_gripper_controllers.yaml               # Definition of controllers for this hardware component, gripper controller in this case
│   ├── launch                                              
│   │   └── bioscara_gripper.launch.py                      # Use this launch file used to start the component stand-alone, including GUI and ros2_control stack.
│   └── package.xml
├── bioscara_gripper_descriptions                           # The description package contains all files to assemble a component description with Xacro 
│   ├── CMakeLists.txt
│   ├── config                                              
│   │   ├── bioscara_gripper_128_parameters.yaml            # Describes static component parameters parsed in the robot description, like joint limits
│   │   └── bioscara_gripper_<variant_X>_parameters.yaml    # Parameters of another gripper variant
│   ├── launch
│   │   └── view.launch.py                                  # Use this launch file to simple view and inspect the assembled component.
│   ├── meshes                                              # Resources for the URDF file
│   ├── package.xml
│   ├── rviz
│   │   └── display.rviz                                    # Saved display configuration for the stand-alone GUIs
│   └── urdf
│       ├── bioscara_gripper_128.ros2_control               # ros2_control hardware information for the 128 mm variant
│       ├── bioscara_gripper_128.urdf                       # Base URDF file containing kinematic, visual and collision info for the 128 mm variant
│       ├── bioscara_gripper_128.xacro                      # Contains the "load_gripper" macro, combining the .urdf and .ros2control parts for the 128 mm variant
│       ├── bioscara_gripper_<variant_X>.ros2_control       
│       ├── bioscara_gripper_<variant_X>.urdf                       
│       ├── bioscara_gripper_<variant_X>.xacro              # Variants of the gripper using the same hardware interface can be added here
│       ├── materials.xacro
│       └── scene.xacro                                     # bringup/launch file uses this scene for the stand-alone start of the component.
├── bioscara_gripper_hardware_driver                        # The hardware driver contains the hardware specific implementation, like PWM generation for the gripper
│   ├── CMakeLists.txt
│   ├── include
│   │   └── bioscara_gripper_hardware_driver
│   ├── package.xml
│   └── src
│       ├── mBaseGripper.cpp                                # Base implementation
│       ├── mGripper.cpp                                    # Hardware implementation
│       └── mMockGripper.cpp                                # Mock implementation for testing
├── bioscara_gripper_hardware_interface                     # The hardware interface abstracts the hardware driver and creates a hardware componen plugin for ros2_control
│   ├── CMakeLists.txt
│   ├── bioscara_gripper_hardware_interface.xml
│   ├── include
│   │   └── bioscara_gripper_hardware_interface
│   ├── package.xml
│   └── src
│       └── gripper_hardware.cpp
└── dalsa_bioscara_grippers                                 # Meta package
```

The name of the gripper component package is written in plural since it can include multiple variants of the bioscara gripper which use the same hardware interface but might have differen geometries.

### Bringup Packages
The bringup packages inside hardware components contain the ros2_control controller configurations for that specific hardware component, as well as a controller manager configuration if the hardware component is launched stand-alone with the launch file included in the bringup package. Stand-alone launch includes the ros2_control stack and can be used for example hardware interface testing, without having to also start higher level applications like MoveIt.

### Description Packages
Description packages are a core building block of the modular principle. It contains robot description files and control configuration files.
A robot description file is an integral part in many ROS2 applications. ROS2 uses an XML based format called Unified Robot Description File (URDF) which is "*a format to describe the kinematics, dynamics, and geometries of robots, independently of software program*" [[1]][urdf], in detail this includes a systems:

- Kinematic description (joints and frame definitions)
- Visual properties for viszualization
- Collision properties defining physical boundaries for trajectory generation
- Mass and Inertial properties for physical simulation (not utilized in this project)
- The URDF can be expanded for specific applications, in this project it addtionally includes:
- ros2_contol configuration
  - hardware interface plugin
  - command and state interfaces
  - joints
  - gpios


The URDF files can be combined using Xacro, a XML macro extension. With Xacro a virtual robot assembly can be created, allowing to flexibly match and mix hardware components. A desription of the Xacro macros follows in the next section.

The description package also contains a stand-alone *scene* and launch file which can be used to simply display the geometry visuals and collision bodies, test joint limits using joint state broadcaster and frame transformations.


[urdf]: https://vbn.aau.dk/ws/files/710175796/main.pdf	"Understanding URDF: A Dataset and Analysis"


### Dynamic Robot Assembly
The entire robot is assembled in the *scene_descriptions* package, in particular the *load_robot* macro defined in *scene_descriptions/robot.xacro".
The diagram below tries to viszualize the different Xacro macros that make up the robot. The robot can be modified simply either by passing diffferent arguments to the macros, loading different parameters from the paramter files or swapping entire macros. For example the *scene.xacro* file takes a gripper macro path as an argument, thus an entirely different gripper can be loaded as long as it supports the same input arguments. Alternatively one can decide to not mount a gripper at all to the robot by passing an empty gripper macro name.


```{mermaid}
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


After assembling the bioscara robot in its default state the following kinematic chain is created:

```{mermaid}
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
## Root Level Scene Packages

### Scene Description
The main *scene.xacro* file describes the robot and its environment, in this case a table top. It loads the *load_robot* macro defined in the *robot.xacro* file. The resulting robot has been described in the [Robot Assembly section](#dynamic-robot-assembly).

## Scene Bringup
Contains the launch files of a specific robot configuration. 

:::{tip}
This package contains the main launch file to start a robot configuration including the ros2_control stack using the *\<RobotConfiguration\>.launch.py* launch file.
Start Bioscara with the currently mounted 128 mm gripper, execute `ros2 launch scene_bringup bioscara_arm_gripper128.launch.py use_mock_hardware:=true/false gui:=true/false`.
:::

:::{note}
This launch files might be superseeded by a single launch file that additionally launches the MoveIt move_group from a hardware specific dalsa_moveit_configurations package.
:::