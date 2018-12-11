#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import ik_4dof

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


		############################ Method : Joint values (Go home!)############################


		# We can get the joint values from the group and adjust some of the values:

		# Go home!!!
		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = 0   		# arm_shoulder_pan_joint
		joint_goal[1] = -pi*5/13   	# arm_shoulder_lift_joint
		joint_goal[2] = pi*3/4   	# arm_elbow_flex_joint
		joint_goal[3] = pi/3   		# arm_wrist_flex_joint
		joint_goal[4] = 0   		# gripper_joint

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		move_group.stop()

		############################ Method : Using IK to calculate joint value ############################

		# After determining a specific point where arm should move, we input x,y,z,degree to calculate joint value for each wrist. 

		pose_goal = Pose()
		pose_goal.position.x = 0.141
		pose_goal.position.y = 0.020
		pose_goal.position.z = 0.042
		# ik_4dof.ik_solver(x, y, z, degree)
		joint_value = ik_4dof.ik_solver(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, -90)

		for joint in joint_value:
			joint = list(joint)
			# determine gripper state
			joint.append(0)
			try:
				move_group.go(joint, wait=True)
			except:
				rospy.loginfo(str(joint) + " isn't a valid configuration.")

		# Reference: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
	def onShutdown(self):
		rospy.loginfo("Shutdown.")

if __name__ == '__main__': 
	rospy.init_node('moveit_tutorial',anonymous=False)
	rospy.sleep(2)
	moveit_tutorial = moveit_tutorial()
	rospy.on_shutdown(moveit_tutorial.onShutdown)
	rospy.spin()
