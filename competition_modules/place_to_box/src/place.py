#!/usr/bin/env python

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi,sqrt
from std_msgs.msg import String,Float64, Bool
from moveit_commander.conversions import pose_to_list
import rospy
import tf
from place_to_box.srv import *


class place_node(object):
	def __init__(self):
		self.listener = tf.TransformListener()
		self.node_name = "place_node"
		check = True
		while(check):
			check = False
			try:
				self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
			except:
				print "moveit server isn't open yet"
				check = True
		self.move_group_arm.set_goal_tolerance(0.02)

		self.pub_gripper = rospy.Publisher("/gripper_joint/command",Float64,queue_size=1)

			#self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		self.place_srv = rospy.Service("place_to_box", tag, self.transform)
		self.place_srv = rospy.Service("home_place",home, self.home)

		#self.sub_control = rospy.Subscriber("~place_control", Bool, self.transform, queue_size=1)
		#self.pub_trans = rospy.Publisher("~colorSegment", Image, queue_size=1)
	def transform(self,req):
		br = tf.TransformBroadcaster()
		tag = "tag_" + str(req.tag_id)
		try:
			now = rospy.Time.now()
			self.listener.waitForTransform('pi_camera', tag, now, rospy.Duration(3.0))
			(trans, rot) = self.listener.lookupTransform('pi_camera', tag , now)
			

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, \
			tf.Exception):
			rospy.loginfo("Tag not found!")
			return tagResponse("Tag not found!")
		#tag_gripper = Quaternion(-0.694,-0.010, 0.719, 0.007)
		#car_tag = Quaternion(rot[3],rot[0], rot[1], rot[2])
		#car_gripper = car_tag * tag_gripper

		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.x = -0.706
		pose_goal.orientation.y = -0.001
		pose_goal.orientation.z = 0.708
		pose_goal.orientation.w = -0.001
		
		pose_goal.position.x = trans[2] - 0.185
		pose_goal.position.y = - trans[0]
		pose_goal.position.z = trans[1] + 0.01


		print "Your box's TF : " , pose_goal 

		self.move_group_arm.set_pose_target(pose_goal)
		plan = self.move_group_arm.go(wait=True)
		rospy.loginfo("Move arm to target box")
		self.move_group_arm.stop()
		self.move_group_arm.clear_pose_targets()
                rospy.sleep(0.5)
		grip_data = Float64()
		grip_data.data = 0.5
		self.pub_gripper.publish(grip_data)
                rospy.sleep(2)
		#joint_goal = self.move_group_gripper.get_current_joint_values()
		#joint_goal[0] = 0
		#self.move_group_gripper.go(joint_goal, wait=True)
		#rospy.loginfo("Place object into target box")


		joint_goal = self.move_group_arm.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = pi/3
		joint_goal[2] = pi/3
		joint_goal[3] = 0
		joint_goal[4] = 0
		self.move_group_arm.go(joint_goal, wait=True)
		grip_data.data = 2.0 
		self.pub_gripper.publish(grip_data)
		rospy.loginfo("End process")
		return tagResponse("Process Successfully")

	def home(self,req):
		joint_goal = self.move_group_arm.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -pi*5/13
		joint_goal[2] = pi*3/4
		joint_goal[3] = pi/3
		joint_goal[4] = 0
		self.move_group_arm.go(joint_goal, wait=True)
		return homeResponse("Home now!")		
   
	def onShutdown(self):
		self.loginfo("Shutdown.")

if __name__ == '__main__': 
	rospy.init_node('place_node',anonymous=False)
	rospy.sleep(120)
        place_node = place_node()
	rospy.on_shutdown(place_node.onShutdown)
	rospy.spin()

