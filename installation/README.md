# Setup on the Raspberry Pi
> [!CAUTION]
>
> This list is under development and not complete (yet)

- Install Ubuntu Server 24.04 LTS
- Install openssh-server
- Run the scripts provided in [/scripts](scripts)



## Other Dependencies
The order of managing dependenices is according to [this](https://ros2-quality-assurance.readthedocs.io/en/released/tutorials/dependencies.html) guideline the following:
1. **rosdep**: Installs missing dependecies specified in the packages via the systems package manager.
2. **vcstool**: Specify further source code repositoryies in a repository file, vcstool will then retrieve the repository and it can then be built with colcon. 
3. **other**

### ROS2 dependencies
To install ROS2 package dependencies navigate to the workspace:
```bash
cd lib/ros2_ws
```

**First:**
Fetch all dependecies that are not available as binaries using the *vcstool*.
From the workspace execute:
```bash
vcs import --recursive src < req.repos
```
This will pull the repositories specified in *req.repos* into the directories also specified in the file.

> [!NOTE]
>
> *vcstool* is found in many ROS2 packages to import dependencies that are not in a ROS or debian repository from a repository file. 
>
> **Installation**:
>
> ```bash
> sudo apt install python3-vcstool
> ```


**Second:**

Install all packages that can be resolved through `rosdep` (all packages that have been released to the ROS2 package ecosystem and some debian packages):
```bash
sudo apt update
rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO
```
This command will recursively scan every package in the workspace for the `<depend/>` key and install missing packages.

Then dont forget to build the workspace:
```bash
sudo apt remove ros-$ROS_DISTRO-moveit*
rm -rf build/ install/ log/
MAKEFLAGS="-j1 -l1" colcon build --mixin release --executor sequential --symlink-install > log.out &
```

## Development Purposes:
### Install [PlotJuggler](https://github.com/facontidavide/PlotJuggler)
A very powerfull tool to to display data
```bash
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
```

### Install RQT tools
```bash
sudo apt install ros-jazzy-rqt*
```

### For debugging
```bash
sudo apt install xterm gdb gdbserver
```
