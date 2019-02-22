# -*- coding: utf-8 -*-
"""
Created on Fri Feb 22 11:42:43 2019

@author: student
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


wheel_radius = 0.1
robot_radius = 1.0


# computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_r - c_l) / (2 * robot_radius)
    return (v, a)


# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
    c_l = v - (robot_radius * a)
    c_r = v + (robot_radius * a)
    w_l = c_l / wheel_radius
    w_r = c_r / wheel_radius
    return (w_l, w_r)


# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
    return inverse_kinematics(t.linear.x, t.angular.z)

    
    
class wheel_test:
    
    def __init__(self):
        rospy.init_node('wheel_test', anonymous=True)
	#self.
        self.sub = rospy.Subscriber('/wheel_vel_left', Float32, self.callback)
        #print("test")
    def callback(self,data):
       	 w_l = data.data
	 w_r = 0.0
	 (v, a) = forward_kinematics(w_l, w_r)  
	 #print "v = %f,\ta = %f" % (v, a)
	 pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
         t = Twist()
	 t.linear.x = v
	 t.angular.z = a
	 
	 pub.publish(t)

if __name__ == "__main__":
    
    wheel_test()
    
    #pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
    #sub = rospy.Subscriber('/wheel_vel_left', Float32, callback)
    rospy.spin()

    (w_l, w_r) = inverse_kinematics(0.0, 1.0)
    print "w_l = %f,\tw_r = %f" % (w_l, w_r)

    (v, a) = forward_kinematics(w_l, w_r)
    print "v = %f,\ta = %f" % (v, a)

    (w_l, w_r) = inverse_kinematics_from_twist(t)
    print "w_l = %f,\tw_r = %f" % (w_l, w_r)
    
    #in terminal: rostopic pub /wheel_vel_left std_msgs/Float32 "data: 1.0" -r 10
    
    