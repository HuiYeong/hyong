#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	print('name %s' %data.data)

def listener():
    rospy.init_node('kunsan', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
