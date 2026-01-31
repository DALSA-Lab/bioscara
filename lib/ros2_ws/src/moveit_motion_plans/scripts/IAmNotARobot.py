#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from math import pi, cos, sin

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3Stamped, Vector3, PoseStamped, Pose, Point, Quaternion
from moveit.task_constructor import core, stages
# from tf_transformations import euler_from_quaternion

import rclcpp

rclcpp.init()
node = rclcpp.Node("IAmNotARobot")

group = "arm"

# Cartesian and joint-space interpolation planners
cartesian = core.CartesianPath()
jointspace = core.JointInterpolationPlanner()
pipeline = core.PipelinePlanner(node, "ompl", "RRTConnect") 


task = core.Task()
task.name = "I Am Not A Robot"
task.loadRobotModel(node)

# start from current robot state
task.add(stages.CurrentState("current state"))

## Disable collisions between jaws and target object
mps = stages.ModifyPlanningScene("Allow Collision")
mps.allowCollisions("target", ["right_jaw_link", "left_jaw_link"], True)
task.add(mps)


# Calculate the inverse kinematics for the current robot state
generatePose = stages.GeneratePose("generate pose")
# spwan a pose whenever there is a solution of the monitored stage
generatePose.setMonitoredStage(task["current state"]) #required
# Define a target pose
pose = PoseStamped()
pose.header.frame_id = "target"
# a = 0
# pose.pose.orientation.z = sin(a/2)
# pose.pose.orientation.w = cos(a/2)
generatePose.pose = pose

connect = stages.Connect("connect1", [(group,pipeline)])
task.add(connect)

# approach, relative cartesian
## reduce the speed
cartesian.max_velocity_scaling_factor = 0.1
move = stages.MoveRelative("click", cartesian)
move.group = group
header = Header(frame_id="target")
# header = Header(frame_id="world")
move.setDirection(Vector3Stamped(header=header, vector=Vector3(x=0.05, y=0.0, z=-0.05)))
task.add(move)



# Compute IK of target pose
computeIK = stages.ComputeIK("compute IK", generatePose)
computeIK.group = group  # Use the group-specific IK solver
computeIK.ik_frame = PoseStamped(header=Header(frame_id="tool"))
props = computeIK.properties
# derive target_pose from child's solution, required
props.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ["target_pose"])
task.add(computeIK)


# Retreat
move = stages.MoveRelative("retreat", cartesian)
move.group = group
header = Header(frame_id="target")
move.setDirection(Vector3Stamped(header=header, vector=Vector3(x=-0.05, y=0.0, z=0.05)))
task.add(move)

connect = stages.Connect("go back", [(group,pipeline)])
task.add(connect)

task.add(stages.CurrentState("current state"))

if task.plan():
    task.publish(task.solutions[0])
