#!/usr/bin/env python3

import rospy
from std_msgs.msg import String  # Change the message type according to your topic

def callback(data):
    rospy.loginfo("Received data: %s", data.data)

def listener():
    rospy.init_node('simple_subscriber', anonymous=True)
    rospy.loginfo("Node starterd ")

    # Change "/example_topic" to the actual topic you want to subscribe to
    rospy.Subscriber("joint_angles", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
