#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3Stamped, Vector3, PoseStamped, Pose, Point, Quaternion
from moveit.task_constructor import core, stages
import rclcpp

rclcpp.init()
node = rclcpp.Node("PickPlace")

arm = "arm"
eef = "gripper"
object_name = "plate"

# Cartesian and joint-space interpolation planners
cartesian = core.CartesianPath()
jointspace = core.JointInterpolationPlanner()
pipeline = core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault")


task = core.Task()
task.name = "Pick + Place"
task.loadRobotModel(node)

# TODO: Does not seem to work, investigate if user error or MTC
allowCollision = stages.ModifyPlanningScene("Allow Collision (Plate, Table)")
allowCollision.allowCollisions([object_name],["table"],True)
task.add(allowCollision)

# start from current robot state
startState = stages.CurrentState("start state")
task.add(startState)

moveToStandby = stages.MoveTo("Move to Standby",pipeline)
moveToStandby.ik_frame = PoseStamped(header=Header(frame_id="tool"))
moveToStandby.group = arm
standbyPose = PoseStamped(header=Header(frame_id="world"))
standbyPose.pose.position.y = 0.4
standbyPose.pose.position.z = 0.2
standbyPose.pose.orientation.z = 1.0
standbyPose.pose.orientation.w = 1.0
moveToStandby.setGoal(standbyPose)
# task.add(moveToStandby)

# Connect start state to approach point.
task.add(stages.Connect("Go to Approach", [(arm, pipeline),(eef,jointspace)]))

# The grasp generator spawns a set of possible grasp poses around the object
grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
grasp_generator.angle_delta = math.pi / 2
grasp_generator.pregrasp = "open"
grasp_generator.grasp = "close_75"
# grasp_generator.setPreGraspPose({"gripper":0.12}) TODO: This can be supported with some changes in MTC
# grasp_generator.setGraspPose({"gripper":0.09})
grasp_generator.setMonitoredStage(task["start state"])  # Generate solutions for all initial states


# The simpleGrasp stage combines ik calculation with motion plan generation 
# for opening and closing the end effector, as well as attaching the object
#  to the robot and disabling collision.
simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp")
# Set frame for IK calculation in the center between the fingers
ik_frame = PoseStamped()
ik_frame.header.frame_id = "tool"
ik_frame.pose.position.z = 0.0  # tune to get height correct
simpleGrasp.setIKFrame(ik_frame)

# Pick container comprises approaching, grasping (using SimpleGrasp stage), and lifting of object
pick = stages.Pick(simpleGrasp, "Pick")
pick.eef = eef
pick.object = object_name

# Twist to approach the object
approach = TwistStamped()
approach.header.frame_id = "tool"
approach.twist.linear.z = -1.0
pick.setApproachMotion(approach, 0.03, 0.1)

# TODO: Does NOT work, crash. MTC problem, this comes straight from example
# pick.cartesian_solver.max_velocity_scaling_factor = 0.1
# print(pick.cartesian_solver.max_velocity_scaling_factor)

# Twist to lift the object
lift = TwistStamped()
lift.header.frame_id = "tool"
lift.twist.linear.z = 1.0
pick.setLiftMotion(lift, 0.03, 0.1)

# Add the pick stage to the task's stage hierarchy
task.add(pick)

# Connect the Pick stage with the following Place stage
con = stages.Connect("Transfer", [(arm, pipeline),(eef,jointspace)])
task.add(con)

# Define the pose that the object should have after placing
placePose = PoseStamped()
placePose.header = Header(frame_id="world")
placePose.pose.position.x = -0.2
placePose.pose.position.y = 0.4
placePose.pose.position.z = 0.011

# Generate Cartesian place poses for the object
place_generator = stages.GeneratePlacePose("Generate Place Pose")
place_generator.setMonitoredStage(task["Pick"])
place_generator.object = object_name
place_generator.pose = placePose

# The SimpleUnGrasp container encapsulates releasing the object at the given Cartesian pose
simpleUnGrasp = stages.SimpleUnGrasp(place_generator, "UnGrasp")

# Place container comprises placing, ungrasping, and retracting
place = stages.Place(simpleUnGrasp, "Place")
place.eef = eef
place.object = object_name
place.eef_frame = "tool"

# Twist to retract from the object
retract = TwistStamped()
retract.header.frame_id = "tool"
retract.twist.linear.z = 1.0
place.setRetractMotion(retract, 0.03, 0.1)

# Twist to place the object
placeMotion = TwistStamped()
placeMotion.header.frame_id = "tool"
placeMotion.twist.linear.z = -1.0
place.setPlaceMotion(placeMotion, 0.03, 0.1)

# Add the place pipeline to the task's hierarchy
task.add(place)

moveToStandby = stages.MoveTo("Move to Standby",pipeline)
moveToStandby.ik_frame = PoseStamped(header=Header(frame_id="tool"))
moveToStandby.group = arm
moveToStandby.setGoal(standbyPose)
task.add(moveToStandby)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(3600)