#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs import LaserScan

def callback(data):
	print('name %s' %data.data)

def listener():
    rospy.init_node('rp', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
