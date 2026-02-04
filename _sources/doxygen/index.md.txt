# Bioscara C++ API Documentation
This is the API documentation for the Bioscara robotic control middleware based on the ROS2 framework.

Please find the key components below:
- bioscara_joint_firmware: contains the joint firmware and its associtated helper classes.
- bioscara_hardware_drivers: contains the BaseJoint, Joint and MockJoint drivers as well as the BaseGripper, Gripper and MockGripper drivers.
- bioscara_hardware_interfaces: contains the ros2_control hardware interfaces that implement the Bioscara arm and gripper hardware components in the BioscaraArmHardwareInterface and BioscaraGripperHardwareInterface respectively.
- bioscara_rviz_plugin: contains the BioscaraPanel Rviz GUI panel.
- single_trigger_controller::SingleTriggerController: contains the documentation for the SingleTriggerController.
