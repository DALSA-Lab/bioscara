# Installation

<!-- Merge dev_machine and Raspberry Pi Readme here -->

## Robot Controller (Raspberry Pi 4B 4GB)




:::{note}
Building MoveIt2 from source was only necessary during development since a bugfix has not been released yet. If MoveIt2 with a version >= 2.12.4 is available as a binary, it is greatly recommended to install it as a binary instead. Building MoveIt from source is very time consuming (~10 h on the Raspberry Pi). 
:::


## Control PC
The control PC is a desktop PC running the Ubuntu 24.04 LTS Desktop operating system. Its purpose is to be the primary control interface for the user. It is in the same network as the robot controller and thus the ROS2 nodes on the robot controller and control PC can communicate with each other. Recommended use case (at the current development state):
- Run the RViz GUI with the Bioscara Panel, MoveIt and MTC Panel to control the hardware state, manual trajectory generation and sequence inspection
- Since the MTC is not realtime-critical, it can also run on the control PC

### OS Installation
Install the latest Ubuntu 24.04 LTS Desktop release according to [the offical guide](https://ubuntu.com/download/desktop?version=24.04).

Create the following user:
- **User:** *scara-dev*
- **Password:** *dtubio*

### Install ROS2
Use the *installation/scripts/ROS2-Jazzy_install.sh* to install ROS2. The installation requires root priviliges, enter the password when prompted:

```bash
bash installation/scripts/ROS2-Jazzy_install.sh
```

<!-- needs full installation as well -->