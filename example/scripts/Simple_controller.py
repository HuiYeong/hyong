#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos,sin,pi
from geometry_msgs.msg import Point32

class Simple_controller:
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.motor_pub = rospy.Publisher("commands/motor/speed", Float64, queue_size=1)
        self.servo_pub = rospy.Publisher("commands/servo/position", Float64, queue_size=1)
        self.pcd_pub = rospy.Publisher("laser2pcd", PointCloud, queue_size=1)

        while not rospy.is_shutdown():
            rospy.spin()

    def laser_callback(self,msg):
        pcd=PointCloud()
        motor_msg=Float64()
        servo_msg=Float64()
        pcd.header.frame_id=msg.header.frame_id
        angle=0
        for r in msg.ranges:
            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)
            angle=angle+(1.0/180*pi)
            if r<12:
                pcd.points.append(tmp_point)

        count=0
        left = 0
        right = 0
        for point in pcd.points:
            if point.x > 0 and point.x < 1 and point.y > 0 and point.y < 1:
                count=count+1
            if point.y < 0 and point.y > -1 and point.x > 0 and point.x < 1:
                left += 1
            if point.y > 0 and point.y < 1 and point.x > 0 and point.x < 1:
                right += 1
        
        if count > 20: 
            print("stop")
            motor_msg.data = 0
            #if left > right:
                #servo_msg.data = 0.15
                #count = 0
            #elif right > left:
                #servo_msg.data = 0.85
                #count = 0
        else :
            motor_msg.data = 5000

        print("count")
        print(count)
        print("left")
        print(left)
        print("right")
        print(right)
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)
        self.pcd_pub.publish(pcd)

if __name__ == "__main__":
    try:
        test_track = Simple_controller()
    except rospy.ROSInterruptException:
        pass
