import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "gripper"
move_group = moveit_commander.MoveGroupCommander(group_name)

#move_group.setPoseReferenceFrame(arm_base_link)


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
print "============ Printing robot state"
print robot.get_current_state()
print ""

joint_goal = move_group.get_current_joint_values()
print(joint_goal)
joint_goal[0] = 0.01
print(joint_goal)
#joint_goal[1] = 0
#joint_goal[2] = 0
#joint_goal[3] = 0.2
#joint_goal[4] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

move_group.set_goal_tolerance(0.02)
#move_group.set_goal_position_tolerance(0.1)
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
