#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry

def turtlebot1_pose_callback(data):
    turtlebot1_x=data.pose.pose.position.x
    turtlebot1_y=data.pose.pose.position.y
    turtlebot1_z=data.pose.pose.position.z
    global z1
    z1= np.array([turtlebot1_x, turtlebot1_y, turtlebot1_z])
    print("inside1", z1)

def turtlebot2_pose_callback(data):
    turtlebot2_x=data.pose.pose.position.x
    turtlebot2_y=data.pose.pose.position.y
    turtlebot2_z=data.pose.pose.position.z
    z2 = np.array([turtlebot2_x, turtlebot2_y, turtlebot2_z])
    z21 = np.linalg.norm(z2-z1)
    print("inside2", z21)

def turtlebot3_pose_callback(data):
    turtlebot3_x=data.pose.pose.position.x
    turtlebot3_y=data.pose.pose.position.y
    turtlebot3_z=data.pose.pose.position.z
    z3 = np.array([turtlebot3_x, turtlebot3_y, turtlebot3_z])
    z31 = np.linalg.norm(z3-z1)
    print("inside3", z31)

def turtlebot4_pose_callback(data):
    turtlebot4_x=data.pose.pose.position.x
    turtlebot4_y=data.pose.pose.position.y
    turtlebot4_z=data.pose.pose.position.z
    z4 = np.array([turtlebot4_x, turtlebot4_y, turtlebot4_z])
    z41 = np.linalg.norm(z4-z1)
    print("inside4", z41)

if __name__=="__main__":
    rospy.init_node("mr_target_tracking_node")
    rate = rospy.Rate(5)
    print("inside motion_predictor")
    rospy.Subscriber('/turtlebot1/odom', Odometry, turtlebot1_pose_callback)
    rospy.Subscriber('/turtlebot2/odom', Odometry, turtlebot2_pose_callback)
    rospy.Subscriber('/turtlebot3/odom', Odometry, turtlebot3_pose_callback)
    rospy.Subscriber('/turtlebot4/odom', Odometry, turtlebot4_pose_callback)
    rospy.spin()
    # p(x_t|z_1:t)  = ? particle filter