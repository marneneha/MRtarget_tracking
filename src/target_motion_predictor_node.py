#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np

def turtlebot1_pose_callback(msg):
    turtlebot1_x=msg.pose.position.x
    turtlebot1_y=msg.pose.position.y
    turtlebot1_z=msg.pose.position.z

def turtlebot2_pose_callback(msg):
    turtlebot2_x=msg.pose.position.x
    turtlebot2_y=msg.pose.position.y
    turtlebot2_z=msg.pose.position.z
    z21 = np.norm(z2,z1)

def turtlebot3_pose_callback(msg):
    turtlebot3_x=msg.pose.position.x
    turtlebot3_y=msg.pose.position.y
    turtlebot3_z=msg.pose.position.z
    z31 = np.norm(z3,z1)

def turtlebot4_pose_callback(msg):
    turtlebot4_x=msg.pose.position.x
    turtlebot4_y=msg.pose.position.y
    turtlebot4_z=msg.pose.position.z
    z41 = np.norm(z4,z1)

if __name__=="__main__":
    rospy.init_node("mr_target_tracking_node")
    rate = rospy.Rate(5)
    turtlebot1_sub = rospy.Subscriber('/turtlebot1/odom', queue_size=10 turtlebot1_pose_callback)
    turtlebot2_sub = rospy.Subscriber('/turtlebot2/odom', queue_size=10 turtlebot2_pose_callback)
    turtlebot3_sub = rospy.Subscriber('/turtlebot3/odom', queue_size=10 turtlebot3_pose_callback)
    turtlebot4_sub = rospy.Subscriber('/turtlebot4/odom', queue_size=10 turtlebot4_pose_callback)
    # p(x_t|z_1:t)  = ? particle filter