#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import pcl

class VelocityProfile:
    def __init__(self):
        rospy.Subscriber('radar',PointCloud2,self.radarCallback)

    def radarCallback(self,msg):
        print(msg.header)
        clusters = PointCloud2()
        clusters.data = msg.data

        pass

if __name__ == '__main__':
    rospy.init_node('velocity_profile')
    velProfile = VelocityProfile()
    rospy.spin()