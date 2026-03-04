#!/bin/bash

# Parts of this script have been created by AI


p_ROS_DOMAIN_ID=41
p_ROS_VERSION=ros-desktop

# Install necessary packages
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y

# Add ROS repository key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
sudo apt update

# Install the specified ROS version
if [ "$p_ROS_VERSION" == "ros-base" ]; then
    echo "Installing ros-jazzy-base"
    sudo apt install ros-jazzy-ros-base -y
elif [ "$p_ROS_VERSION" == "ros-desktop" ]; then
    echo "Installing ros-jazzy-ros-desktop"
    sudo apt install ros-jazzy-desktop -y
fi

# Source ROS setup script
source /opt/ros/jazzy/setup.bash

# Add source command to .bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=$p_ROS_DOMAIN_ID" >> ~/.bashrc

# Install colcon extensions
sudo apt install python3-colcon-common-extensions -y
sudo apt-get install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# rosdep
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update

# vcstool
sudo apt install python3-vcstool -y

# debugging (not required for normal operation)
sudo apt install xterm gdb gdbserver -y


echo "ROS installation completed. Please restart your terminal or run 'source ~/.bashrc' to apply changes."
