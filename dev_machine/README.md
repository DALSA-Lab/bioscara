# Setup on the Dev machine
- Install openssh-server
        ```bash
    sudo apt install openssh-server
    ```
- enable ssh on boot
    ```bash
    sudo systemctl enable ssh
    ```
- install net-tools

## install ROS2
using this [script](../Raspberry/scripts/ROS2-Jazzy_install.sh) with arguments domain ID = 41 and version ROS-Desktop

## Install ROS2_control
Although not neccessarily needed, this package comes with some handy tools that might be usefull.
```bash
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers
```

In my case xacro was not installed, had to manually install it using
```bash
sudo apt install ros-$ROS_DISTRO-xacro
```

## Network Configuration
The dev-machine is connected to the robot via a local private network via a switch like displayed [here](../Raspberry/README.md#network-configuration).

### Assign a static IP
create the */etc/netplan/99_config.yaml* file with following content:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eno1:
      addresses:
        - 10.10.10.3/24
      routes:
        - to: default
          via: 10.10.10.1
          metric: 700 # Increase the metric so that a the wifi connection (metric: 600) is preffered for internet traffic
      nameservers:
          addresses:
            - 8.8.8.8
            - 8.8.4.4
```
to apply the changes execute:
```bash
sudo netplan apply
```

### Create a static hostname entry
in the */etc/hosts* add the following line:
```
10.10.10.3 scara-dev
```

## Development Purposes:
### Install [PlotJuggler](https://github.com/facontidavide/PlotJuggler)
A very powerfull to to display data
```bash
sudo snap install plotjuggler
```