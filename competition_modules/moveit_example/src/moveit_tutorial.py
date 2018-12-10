#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class moveit_tutorial(object):
	def __init__(self):
		check = True
		# you need to check if moveit server is open or not
		while(check):
			check = False
			try:
				# Instantiate a MoveGroupCommander object. This object is an interface to a planning group 
				# In our case, we have "arm" and "gripper" group

				move_group = moveit_commander.MoveGroupCommander("arm")
			except:
				print "moveit server isn't open yet"
				check = True





		# First initialize moveit_commander
		moveit_commander.roscpp_initialize(sys.argv)

		# Instantiate a RobotCommander object. 
		# Provides information such as the robot's kinematic model and the robot's current joint states
		robot = moveit_commander.RobotCommander()

		# Instantiate a PlanningSceneInterface object. 
		# This provides a remote interface for getting, setting, 
		# and updating the robot's internal understanding of the surrounding world
		scene = moveit_commander.PlanningSceneInterface()

		# Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
				                                moveit_msgs.msg.DisplayTrajectory,
				                                queue_size=20)

		# We can get the name of the reference frame for this robot:
		planning_frame = move_group.get_planning_frame()
		print "============ Planning frame: %s" % planning_frame

		# We can also print the name of the end-effector link for this group:
		eef_link = move_group.get_end_effector_link()
		print "============ End effector link: %s" % eef_link

		# We can get a list of all the groups in the robot:
		group_names = robot.get_group_names()
		print "============ Available Planning Groups:", robot.get_group_names()

		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		print "============ Printing robot state", robot.get_current_state()
		print ""


		############################ Method 1: Joint values ############################


		# We can get the joint values from the group and adjust some of the values:
		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = 0   # arm_shoulder_pan_joint
		joint_goal[1] = 0   # arm_shoulder_lift_joint
		joint_goal[2] = 0   # arm_elbow_flex_joint
		joint_goal[3] = 0   # arm_wrist_flex_joint
		joint_goal[4] = 0   # gripper_joint

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		move_group.stop()



		############################ Method 2: Pose planing ############################


		# We can plan a motion for this group to a desired pose for the end-effector:

		# Frist, We need to set tolerance value during RRT planning for pose, otherwise RRT planner won't be albe to plan.
		move_group.set_goal_tolerance(0.02)
		# move_group.set_goal_position_tolerance(???)
		# move_group.set_goal_orientation_tolerance(???)

		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = -0.003
		pose_goal.orientation.x = -0.705
		pose_goal.orientation.y = 0.0
		pose_goal.orientation.z = 0.709

		pose_goal.position.x = 0.205
		pose_goal.position.y = 0.071
		pose_goal.position.z = 0.004

		move_group.set_pose_target(pose_goal)
		plan = move_group.go(wait=True)

		# Calling `stop()` ensures that there is no residual movement
		move_group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		move_group.clear_pose_targets()


		############################ Method 3: Waypoint ############################


		# You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through

		waypoints = []
		scale = 1
		wpose = move_group.get_current_pose().pose
		wpose.position.z -= scale * 0.1  # First move up (z)
		wpose.position.y += scale * 0.2  # and sideways (y)
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.y -= scale * 0.1  # Third move sideways (y)
		waypoints.append(copy.deepcopy(wpose))

		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0,
		# ignoring the check for infeasible jumps in joint space, which is sufficient
		# for this tutorial.
		(plan, fraction) = move_group.compute_cartesian_path(
				                   waypoints,   # waypoints to follow
				                   0.01,        # eef_step
				                   0.0)         # jump_threshold



		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = robot.get_current_state()
		display_trajectory.trajectory.append(plan)
		# Publish
		display_trajectory_publisher.publish(display_trajectory)

		# Use execute if you would like the robot to follow the plan that has already been computed
		move_group.execute(plan, wait=True)

		# Reference: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html

if __name__ == '__main__': 
	rospy.init_node('moveit_tutorial',anonymous=False)
	rospy.sleep(120)
        moveit_tutorial = moveit_tutorial()
	rospy.on_shutdown(moveit_tutorial.onShutdown)
	rospy.spin()
