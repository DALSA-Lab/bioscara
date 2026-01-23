# Bioscara - DALSA DIY SCARA robot arm
This repository serves to collect all information regarding DALSAs DIY robot arm Bioscara.
The latest version is based on an earlier version which can be found under the "bioscara_v1" branch. It is updated by replacing the MKS SERVO42C stepper drivers with the Ustepper S32 drivers and removing the 3D printer driver board. A custom firmware is developed and ROS2 is deployed as the middleware for motion control.

## Repository Structure
The repository structure is based on the DALSA template.

<!-- TODO: structure -->
## Documentation
The C++ source code documentation is generated from comments using doxygen. Doxygen creates the documentation output in multiple formats, HTML for static website and latex for pdf rednering.
Additionally user documentation has also been generated and can be found together with the source code documentation here: https://dalsa-lab.github.io/bioscara/

## Usage
<!-- TODO: link to network setup -->
The robot controller is a Raspberry Pi 4. The RPI is configured to connect to a WIFI network with the the following credentials:  
**SSID**: DALSA_IOT  
**Password**: dalsa_iot  
The easiest way to establish the network is to create a WIFI hotspot with the above credentials, however static IP assignemnt is not possible using Windows Hotspot, and hence the IP address must be checked before any connection attempt.
On the RPI remote login and remote desktop as well as SSH is activated. This way the desktop can be accessed through a RDP software like Windows Remote Desktop or Remina. Be carefull that connecting this way will create a new user session!

Interacting with the robot is handled through ROS2. In the subdirectory a seperate README can be found explaining how to use the hardware.

## TODO
### Mechanical
- [ ]  Update the Fusion model to with all new parts.
- [x] combine old and new stp and stl files
- [x] Fix J3 Pulley slipping by increasing surface roughness
- [x] Gripper Redesign
### Software
- [ ] Finish the TODO list for the Joint Communication Protocol (see the documentation)
- [x] Create URDF file
- [x] Get started with MoveIt
### Documentation
- [ ] Continue this TODO list and Readme
- [ ] Improve User guide with detailed descriptions and up to date 3D models
- [ ] Document mechanical changes
- [ ] Document Ubuntu install and configurations
- [ ] Document ROS installation
- [ ] Explain the scripts
