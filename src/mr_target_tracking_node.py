#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

if __name__=="__main__":
    rospy.init_node("mr_target_tracking_node")
    rate = rospy.Rate(5)
    turtlebot1_vel_pub = rospy.Publisher('/turtlebot1/cmd_vel', Twist, queue_size=10)
    vel = Twist()
    i=100
    while(i):
        if(i>50):
            print("giving the velocity")
            vx=4
            vy=0
            vel.linear.x=vx
            vel.linear.y=vy
            vel.linear.z=0
            vel.angular.x=0
            vel.angular.y=0
            vel.angular.z=0
            turtlebot1_vel_pub.publish(vel)
            i=i-1
            rate.sleep()
        else:
            print("not giving the velocity")
            vx=0
            vy=0
            vel.linear.x=vx
            vel.linear.y=vy
            vel.linear.z=0
            turtlebot1_vel_pub.publish(vel)
            i=i-1
