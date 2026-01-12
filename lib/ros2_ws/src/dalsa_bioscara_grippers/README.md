# Description package for grippers
Currently only the 128 mm custom bioscara gripper is included. To include more grippers simply create their description files on the same directory level. 


TODO:
<!-- https://tree.nathanfriend.com/?s=(%27optEns!(%27fancy!true~fullPath!false~trailingSlash!true~FotDot!false)~C(%27C%27dalsa7sA3-_descriptEns*config5606*launchHview7.launch.py*meshes*rviz*B5.B459540.B40904HA%27)~vGsEn!%271%27)*A33-bEscara70H%3Ccool_othG_-%3E3%20%204.xacF5H-_1286_parametGs.yaml7_grippG9.Fs2_contFl4A%5CnBurdfCsource!EioFroGerH*3%01HGFECBA9765430-* -->



See the [this](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/12e623212e6891a5fcc9af94d67b07e640916394/robotiq_description) repository for inspiration.

```
dalsa_bioscara_grippers/
└── bioscara_gripper_descriptions/
    ├── config/
    │   ├── bioscara_gripper_128_parameters.yaml
    │   └── <cool_other_bioscara_gripper>_parameters.yaml
    ├── launch
    ├── meshes
    ├── rviz
    └── urdf/
        ├── bioscara_gripper_128.urdf.xacro
        ├── bioscara_gripper_128.ros2_control.xacro
        ├── bioscara_gripper_128.xacro
        ├── <cool_other_bioscara_gripper>.urdf.xacro
        ├── <cool_other_bioscara_gripper>.ros2_control.xacro
        └── <cool_other_bioscara_gripper>.xacro
```
