import geometry_msgs.msg
import numpy as np
import rospy
import random
from pyquaternion import Quaternion
import tf
rospy.init_node('test',anonymous=False)

def quaternion_multiply(q1, q0):
    w1 = q1.w
    x1 = q1.x 
    y1 = q1.y
    z1 = q1.z
    w0 = q0.w
    x0 = q0.x 
    y0 = q0.y
    z0 = q0.z
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
a = geometry_msgs.msg.Pose()
b = geometry_msgs.msg.Pose()
c = geometry_msgs.msg.Pose()

a.orientation.w = 0.005
a.orientation.x = 0.812
a.orientation.y = 0.01
a.orientation.z = -0.584

a.position.x = 0.25
a.position.y = 0.005
a.position.z = 0.058

b.orientation.w = 0.005
b.orientation.x = 0.812
b.orientation.y = 0.01
b.orientation.z = -0.584

b.position.x = 0.25
b.position.y = 0.005
b.position.z = 0.058

# br = tf.TransformBroadcaster()
# while True:

#     br.sendTransform((a.position.x, a.position.y, a.position.z),
#                 (a.orientation.x,a.orientation.y,a.orientation.z,a.orientation.w),
#                 rospy.Time.now(),
#                 "a",
#                 "world")
# 

q1 = Quaternion(0.012,1.0, 0.004, 0.016)
q2 = Quaternion(-0.001 ,-0.706, -0.001, 0.708)
q3 = Quaternion(1 ,0 ,0 ,0)
tag_gripper = Quaternion(-0.694,-0.010, 0.719, 0.007)

print(q2 / q1)
#print(tag_gripper.transformation_matrix) 
#print(q1.transformation_matrix)
#print(

#-0.694 -0.005 -0.719 +0.010  pi->gripper


# -0.017 0.055 0.762
# 1.0 0.004 0.016 0.012  car -> tag

# 0.226 0.012 0.064
# -0.706 -0.001 0.708 -0.001  car -> gripper

# 0 0.058 0.411  pi -> tag (y,z,x)
# 1 0 0 0



