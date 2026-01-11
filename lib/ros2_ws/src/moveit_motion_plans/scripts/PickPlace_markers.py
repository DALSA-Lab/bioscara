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


############### PreTask - bringing robot to start state ###############
# Necessary for sequential execution of all soltutions
# otherwise the solutions have discontinous start states
preTask = core.Task()
preTask.name = "GoToStartState"
preTask.loadRobotModel(node)

# start from current robot state
startState = stages.CurrentState("start state")
preTask.add(startState)

moveToStandby = stages.MoveTo("Move to Standby",core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault"))
moveToStandby.group = "arm_gripper"
standbyPose = {"gripper":0.04,"j1":0.58611,"j2":0.051,"j3":2.4804084,"j4":-1.910026}
moveToStandby.setGoal(standbyPose)
preTask.add(moveToStandby)

if preTask.plan():
    preTask.publish(preTask.solutions[0])
    print("Executing PreTask")
    preTask.execute(preTask.solutions[0])
    time.sleep(0.5)
else:
    quit()

############### Main Task ###############

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
moveToStandby.group = "arm_gripper"
moveToStandby.setGoal(standbyPose)
task.add(moveToStandby)

# Connect start state to approach point.
task.add(stages.Connect("Go to Approach", [(arm, pipeline),(eef,jointspace)]))

# The grasp generator spawns a set of possible grasp poses around the object
grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
grasp_generator.angle_delta = math.pi / 2 # TODO: change back to pi/2
grasp_generator.pregrasp = "close_40"
grasp_generator.grasp = "close_40"
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
pick.setApproachMotion(approach, 0.02, 0.03)

# TODO: Does NOT work, crash. MTC problem, this comes straight from example
# pick.cartesian_solver.max_velocity_scaling_factor = 0.1
# print(pick.cartesian_solver.max_velocity_scaling_factor)

# Twist to lift the object
lift = TwistStamped()
lift.header.frame_id = "tool"
lift.twist.linear.z = 1.0
pick.setLiftMotion(lift, 0.02, 0.03)

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
placePose.pose.position.z = 0.04

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
place.setRetractMotion(retract, 0.02, 0.03)

# Twist to place the object
placeMotion = TwistStamped()
placeMotion.header.frame_id = "tool"
placeMotion.twist.linear.z = -1.0
place.setPlaceMotion(placeMotion, 0.02, 0.03)

# Add the place pipeline to the task's hierarchy
task.add(place)

############### Second Pick + Place ###############

task.add(stages.Connect("Go to Approach (2)", [(arm, pipeline),(eef,jointspace)]))

grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose (2)")
grasp_generator.angle_delta = math.pi * 2
grasp_generator.pregrasp = "close_40"
grasp_generator.grasp = "close_40"
# grasp_generator.setPreGraspPose({"gripper":0.12}) TODO: This can be supported with some changes in MTC
# grasp_generator.setGraspPose({"gripper":0.09})
grasp_generator.setMonitoredStage(task["Place"])  # Generate solutions for all initial states

simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp (2)")
ik_frame = PoseStamped()
ik_frame.header.frame_id = "tool"
ik_frame.pose.position.z = 0.0  # tune to get height correct
simpleGrasp.setIKFrame(ik_frame)

pick = stages.Pick(simpleGrasp, "Pick (2)")
pick.eef = eef
pick.object = object_name

approach = TwistStamped()
approach.header.frame_id = "tool"
approach.twist.linear.z = -1.0
pick.setApproachMotion(approach, 0.02, 0.03)

lift = TwistStamped()
lift.header.frame_id = "tool"
lift.twist.linear.z = 1.0
pick.setLiftMotion(lift, 0.02, 0.03)
task.add(pick)

con = stages.Connect("Transfer (2)", [(arm, pipeline),(eef,jointspace)])
task.add(con)

placePose = PoseStamped()
placePose.header = Header(frame_id="world")
placePose.pose.position.x = 0.1
placePose.pose.position.y = 0.35
placePose.pose.position.z = 0.04
placePose.pose.orientation.z = 0.9
placePose.pose.orientation.w = 0.435


# Generate Cartesian place poses for the object
place_generator = stages.GeneratePlacePose("Generate Place Pose (2)")
place_generator.setMonitoredStage(task["Pick (2)"])
place_generator.object = object_name
place_generator.pose = placePose

# The SimpleUnGrasp container encapsulates releasing the object at the given Cartesian pose
simpleUnGrasp = stages.SimpleUnGrasp(place_generator, "UnGrasp (2)")

# Place container comprises placing, ungrasping, and retracting
place = stages.Place(simpleUnGrasp, "Place (2)")
place.eef = eef
place.object = object_name
place.eef_frame = "tool"

# Twist to retract from the object
retract = TwistStamped()
retract.header.frame_id = "tool"
retract.twist.linear.z = 1.0
place.setRetractMotion(retract, 0.02, 0.03)

# Twist to place the object
placeMotion = TwistStamped()
placeMotion.header.frame_id = "tool"
placeMotion.twist.linear.z = -1.0
place.setPlaceMotion(placeMotion, 0.02, 0.03)

# Add the place pipeline to the task's hierarchy
task.add(place)

moveToStandby = stages.MoveTo("Move to Standby (2)",pipeline)
moveToStandby.group = "arm_gripper"
moveToStandby.setGoal(standbyPose)
task.add(moveToStandby)


if task.plan():
    task.publish(task.solutions[0])
    for solution in task.solutions:
        task.execute(solution)
        time.sleep(0.5)
time.sleep(3600)