A bug in Rviz2 introduced somewhere after 14.1.11 makes Moveit Setup assistant crash on loading URDF files. Moveit issue [#3541](https://github.com/moveit/moveit2/issues/3541)
Pull request [#3604](https://github.com/moveit/moveit2/pull/3604) is supposed to fix it. Has not been merged yet (20/11/2025)

For now manually downgrade rviz-common package to 14.1.11:
```bash
wget http://snapshots.ros.org/jazzy/2025-05-23/ubuntu/pool/main/r/ros-jazzy-rviz-common/ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb
sudo dpkg -i ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb
```

or for arm64 architecture:

```bash
wget http://snapshots.ros.org/jazzy/2025-05-23/ubuntu/pool/main/r/ros-jazzy-rviz-common/ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_arm64.deb
sudo dpkg -i ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_arm64.deb
```


<!-- TODO: how to launch  -->
cd to ~/bioscara/ROS2/ros2_scara_ws
source source install/local_setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
