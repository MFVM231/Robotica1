#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def callback1(pose):
    rospy.loginfo("Tortuga 1 en x=%.2f, y=%.2f, θ=%.2f", pose.x, pose.y, pose.theta)
    
def callback2(pose):
    rospy.loginfo("Tortuga 2 en x=%.2f, y=%.2f, θ=%.2f\n", pose.x, pose.y, pose.theta)

def leer_pose():
    rospy.init_node('leer_pose', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, callback1)
    rospy.Subscriber('/turtle2/pose', Pose, callback2)  # Corrección aquí
    rospy.spin()

if __name__ == '__main__':
    leer_pose()