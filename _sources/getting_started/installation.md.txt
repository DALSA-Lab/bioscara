# Installation
This guide describes how to to install and setup all devices to operate the robot. At the first two chapters contain device specific details while the latter can be applied to both the robot controller and a control PC.

:::{important}
This installation guide has not been tested in its entirety. It can thus happen that a prerequisite has been missed, use your own problem solving skills to fill the gap! If you find missing information, include it into this document!
:::


## Robot Controller (Raspberry Pi 4B 4GB) Specific
The following section describes the installation steps specific to the Raspberry Pi robot controller.

### Creating the Boot Image
Using the latest version of the [Raspberry Pi imager](https://www.raspberrypi.com/software/), flash an SD card (16GB, Class 10) with Ubuntu 24.04 LTS Server.
:::{note}
Since the HDMI ports of the Raspberry Pi are no longer accessible when installed inside the robot, follow the next steps to configure remote access.
:::

Flash the SD card with the boot image, you don't need to set custom settings, this will be done in the next step.
**Do not boot the Raspberry Pi yet!**

#### Configuring the Network
When the flashing is finished, copy the *installation/network-config* file to the */boot/* partition of the SD card. 

The *network-config* contains the initial netplan configuration. This file will only be read once per instance (during the first boot). The contents of *network-config* will be copied to */etc/netplan/50-cloud-init.yaml* when installed on the Raspberry Pi [further information](https://docs.cloud-init.io/en/latest/reference/network-config-format-v2.html#netplan-passthrough).

The netplan contains the following configutation:
- Sets the Netplan renderer (the backend for which Netplan is creating the configuration files) to NetworkManager.
- WiFi:
  - Saves an AP with the SSID: "DALSA_IOT" and pre-shared key: "dalsa_iot". See [networking guide](network.md#temporary-wifi-connection) for a description how to use this fallback connection.
- Ethernet:
  - Configures static routing for the "eth0" interface for a LAN. See the network architecture [here](network.md#architecture).
  - The static IP of the robot (the Raspberry Pi) is set to `10.10.10.2`.
  - The ethernet route gets assigned a higher metric to route internet traffic through the WiFi interface if it is connected, since the LAN has not internet access.


#### Troubleshooting

For unknown reasons, while testing theses instructions, it has happened that the Pi was not connecting to the AP nor ethernet. Logging into the Pi with a screen attached showed that the NetworkManager was not installed on the system, this should not happen on a clean installation. If that should happed try one of the following:
- Create a new boot SD card, but edit *network-config* to use 'networkd' (the systemd-networkd network service) instead of 'NetworkManager' as the Netplan renderer. Then, when booted up and connected to a network, install the NetworkManager (`sudo apt install network-manager`). Edit */etc/netplan/50-cloud-init.yaml* back to 'NetworkManager' as renderer and apply `sudo netplan apply`. The network should now be up and managed by the NetworkManager, not systemd-networkd.
- Instead of creating a new boot image, connect to the Pi's shell by attaching display and keyboard and change the renderer to 'networkd', apply `sudo netplan apply`, wait for connection `sudo netplan status -a`, then continue as described above with downloading the NetworkManager.

#### Configuring the User
Copy the *installation/user-data* file to the */boot/* partition of the SD card as well.

The *user-data* file contains mostly configuration for the user *scara* with the password 'dtubio'. The ubuntu cloud-init program manages these configurations during instance deployment. Many settings can be set here, the official README is given for reference:

> *"The user-data of the cloud-init seed. This can be used to customize numerous aspects of the system upon first boot, from the default user, the default password, whether or not SSH permits password authentication, import of SSH keys, the keyboard layout, the system hostname, package installation, creation of arbitrary files, etc. 
> Numerous examples are included (mostly commented) in the default user-data. The format of this file is YAML, and is documented at:<br>
> [https://cloudinit.readthedocs.io/en/latest/topics/modules.html](https://cloudinit.readthedocs.io/en/latest/topics/modules.html) and <br>
> [https://cloudinit.readthedocs.io/en/latest/topics/examples.html](https://cloudinit.readthedocs.io/en/latest/topics/examples.html)"*

You can insert the SD card into the Raspberry Pi and **boot now!** 

Wait for the installation to finish, it will take some time.

### Open Remote Terminal Session
After the installation of the OS has succeeded, we need to log in remotely. The remaining guide assumes you are logged in via ssh to the robot controller.

To do so, first establish a wired or wireless connection to the robot controller as described in the [network guide](network.md) and then log in remotely via ssh:
```bash
ssh scara@<ip-adress>
```

### Connect to the Internet
The following script requires an internet connection. 
Establish an internet connection as described in the [networking guide](network.md#semi-permanent-internet-access-through-dtusecure). 


### Raspi-Config Installation
Installing the Raspi-Config tool on the Ubuntu OS brings some addtional hardware configuation options. 
:::{note}
The exact mechanism and effects of the tool are not fully clear. 
:::

Execute the *installation/scripts/raspi-config_install.sh* script. The source code was provided [here](https://gist.github.com/ryantate13/68882ab4a810369d51da979e37ea9127).

```bash
bash installation/scripts/raspi-config_install.sh
``` 

To apply the changes, restart the robot.

### PWM Activation
In order to use the PWM for servo control, it must be enabled in the */boot/firmware/config.txt* as a "dtoverlay=pwm". However there were issues that the */sys/class/pwm/pwmchip0* could only be used as root. For this reason the *installation/scripts/pwm_activation.sh* registers a udev rule that automatically changes the owner to the "plugdev" group, of which the scara user is a part of. This way the PWM can be controlled without root rights.

```bash
bash installation/scripts/pwm_activation.sh
``` 

### Ensure Low-Latency
To ensure a low-latency operation some settings must be applied. Both of the following settings were already set correctly after the OS installation on the Raspberry Pi. Potentially from the Raspi-config installation(?). Follow the following instructions to ensure they are indeed enabled.

:::{note}
This set of kernel configuration is also set when installing the "Low Latency Ubuntu" (`sudo apt install linux-lowlatency`). But this has not been tested.
:::

#### Preemption Policy 
In mainline Ubuntu the maximum preemption policy is PREEMPT. Ensure the CONFIG_PREEMPT is set:
```bash
cat /boot/<BUILD?> | grep CONFIG_PREEMPT
```
the last known \<BUILD\> was `config-6.8.0-1028-raspi`.

Find more information on the preemption policy [here](https://ubuntu.com/blog/industrial-embedded-systems-ii).

#### Interrupt Timer Resolution
Ensure that a 1000 Hz interrupt timer resolution is set:
```bash
cat /boot/<BUILD?> | grep CONFIG_HZ
```
the last known \<BUILD\> was `config-6.8.0-1028-raspi`.

> *"The timer interrupt handler interrupts the kernel at a rate set by the HZ constant. The frequency affects the timer resolutions as a 100 Hz value for the timer granularity will yield a max resolution of 10ms (1 Hz equating to 1000ms), 250Hz will result in 4ms, and 1000Hz in the best-case resolution of 1ms."* [source](https://ubuntu.com/blog/industrial-embedded-systems-ii)

### Adding the *realtime* Group
Add the user to the realtime group and give it the rights to set higher scheduler priotities as described [here](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html#determinism).

For real-time tasks, a priority range of 0 to 99 is expected, with higher numbers indicating higher priority. By default, users do not have permission to set such high priorities. To give the user such permissions, add a group named realtime and add the user controlling your robot to this group:

```bash
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```
Afterwards, add the following limits to the realtime group in /etc/security/limits.conf:
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
```
The limits will be applied after you log out and in again.

This **concludes** the robot control specific setup.

## Control PC Specific
The control PC is a desktop PC running the Ubuntu 24.04 LTS Desktop operating system. Its purpose is to be the primary control interface for the user. It is in the same network as the robot controller and thus the ROS2 nodes on the robot controller and control PC can communicate with each other. Recommended use case (at the current development state):
- Run the RViz GUI with the Bioscara Panel, MoveIt and MTC Panel to control the hardware state, manual trajectory generation and sequence inspection
- Since the MTC is not realtime-critical, it can also run on the control PC

### OS Installation
Install the latest Ubuntu 24.04 LTS Desktop release according to [the offical guide](https://ubuntu.com/download/desktop?version=24.04).

Create the following user:
- **User:** *scara-dev*
- **Password:** *dtubio*

All following instructions assume you are logged in to the device.

### Network Configuration
First assign a static IP by creating the */etc/netplan/99_config.yaml* file with following content:
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
And the following a simple WPA wifi access point:
```yaml
...
  wifis:
    wlp0s20f3:
      dhcp4: true
      optional: true
      access-points:
        "DALSA_IOT":
          password: "dalsa_iot"
...
```

Then add a static hostname entry by adding the following line in the */etc/hosts* file:
```
10.10.10.3 scara-dev
```

Reboot the machine to apply all changes. Changes to the netplan manager can be applied like this:
```bash
sudo netplan apply
```

### Connect to the Internet
Establish an internet connection as described in the [networking guide](network.md#semi-permanent-internet-access-through-dtusecure). 

## Install I2C dependecies
In order to later compile the ROS2 workspace the only external dependency *lgpio* must be installed. 

Execute the *installation/scripts/i2c_libraryAndTools_install.sh* script:
```bash
bash installation/scripts/i2c_libraryAndTools_install.sh
```



## Install ROS2
Use the *installation/scripts/ROS2-Jazzy_install.sh* to install ROS2. The installation requires root priviliges, enter the password when prompted:

```bash
bash installation/scripts/ROS2-Jazzy_install.sh
```




## Clone the Project Repository
Clone the project repository from Github into the *~/bioscara/* repository:
```bash
git clone https://github.com/DALSA-Lab/bioscara.git
```
## Import further Dependecies
Next we are going to install a lot of dependencies, either as a binary or from source.

### Dependecy Management Tools
The order of managing dependenices is according to [this](https://ros2-quality-assurance.readthedocs.io/en/released/tutorials/dependencies.html) guideline the following:
1. **rosdep**: Installs missing binary dependecies specified in the packages via the systems package manager.
2. **vcstool**: Specify further source code repositoryies in a repository file, `vcstool` will then retrieve the repository and it can then be built with colcon. 
3. **other**: Manually install dependecies. (Already finished after install of *lgpio*)

### Clone further Source Code Repositories
This step pulls all dependecies that are not available as binaries.

Navigate to the ROS2 workspace *lib/ros2_ws*:
```bash
cd lib/ros2_ws
```

`vcstool` is found in many ROS2 packages to import dependencies that are not in a ROS or debian repository from a repository file. It should be automatically installed with the ROS2 install script. If not, its binary name is `python3-vcstool`.

Then invoke the `vcstool` to import the repositories specified in the *lib/ros2_ws/req.repos* file:
```bash
vcs import --recursive src < req.repos
```

This will pull the following repositories:
- [***single_trigger_controller***](https://github.com/DALSA-Lab/ros2_control-single_trigger_controller.git) branch: *main*
  - This repository hosted on the DALSA Github contains the SingleTriggerController used to trigger homing. Since the controller is generic, it is hosted as a seperate repository.
- [***moveit_task_constructor***](https://github.com/moveit/moveit_task_constructor.git) branch: *ros2*
  - The MTC is not available as a binary and must thus be built from source.

:::{important}
The following repositories only need to included if the latest available binary MoveIt2 version is < 2.12.4! All newer version will include a bugfix that was not yet released at the time of writing the tutorial.
**TO-DO:** Remove the following repositories from the *req.repos* file if the binary is available.
:::
- [***moveit2***](https://github.com/DALSA-Lab/moveit2-dalsa.git) branch: *main*
  - MoveIt2 is cloned from a DALSA Fork. Using a fork, we have full control over the versioning and it also includes the changes to the Pilz Motion Controller that allow it to be used with any planning pipeline.
- [***moveit_visual_tools***](https://github.com/moveit/moveit_visual_tools.git) branch: *ros2*
  - Only cloned from source due to installation conflicts with the binary
- [***pick_ik***](https://github.com/PickNikRobotics/pick_ik.git) branch: *main*
  - Only cloned from source due to installation conflicts with the binary

### Install even more Repositories
:::{important}
Do this step ONLY if MoveIt is installed from source! This is NOT necessary if it can be installed as a binary.
:::

Recursively import MoveIt's source code dependencies:
```bash
cd src
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
```

### Binary Dependencies
This steps installs all packages that can be resolved through `rosdep` (all packages that have been released to the ROS2 package ecosystem and some debian packages):
```bash
cd lib/ros2_ws
sudo apt update
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y 
```
This command will recursively scan every package in the workspace for the `<depend/>` key and install missing packages.

As a last step, remove any conflicting MoveIt binaries. This should not be neccessary on a fresh install.
```bash
sudo apt remove ros-$ROS_DISTRO-moveit*
```

### Build the Workspace!
Follow the [building instructions](building.md) to finally compile the entire workspace!
