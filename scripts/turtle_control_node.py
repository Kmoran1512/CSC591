#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from robotics_lab1.msg import Turtlecontrol

prop_msg = Turtlecontrol()
pos_msg = Pose()


def pose_callback(data):
    pos_msg.x = data.x
    pos_msg.y = data.y


def prop_callback(data):
    prop_msg.kp = data.kp
    prop_msg.xd = data.xd


if __name__ == "__main__":
    rospy.init_node("turtle_control_node", anonymous=True)

    prop_sub = rospy.Subscriber("/turtle1/control_params", Turtlecontrol, prop_callback)
    pos_sub = rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    loop_rate = rospy.Rate(10)
    vel_cmd = Twist

    while not rospy.is_shutdown():
        vel_cmd.linear.x = prop_msg.kp * (prop_msg.xd - pos_msg.x)

        vel_pub.publish(vel_cmd)
        loop_rate.sleep()
