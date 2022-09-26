#!/usr/bin/env python
import rospy
import std_msgs.msg 
import os
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge, CvBridgeError
from tqdm import tqdm
import matplotlib.pyplot as plt
from conti_radar.msg import conti_radar 
import pcl

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

def plot_ave(radar_scan):
    x = radar_scan[:,0]
    y = radar_scan[:,1]
    z = radar_scan[:,2]
    v_r = radar_scan[:,4]
    azimuth = np.arctan(y/x)
    elevation = np.arctan(z/np.sqrt(x**2+y**2))
    radar_pub = rospy.Publisher("/radar_vel", PointCloud2,queue_size=100)
    radar_scan[:,0] = azimuth
    radar_scan[:,1] = -v_r
    radar_scan[:,2] = elevation
    header2 = std_msgs.msg.Header()
    header2.stamp = rospy.Time.now()
    header2.frame_id = 'ave'

    fields_radar = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('RCS', 16, PointField.FLOAT32, 1),
            PointField('v_r', 24, PointField.FLOAT32, 1),
            PointField('v_r_compensated', 32, PointField.FLOAT32, 1)]
    radar_pcl = pcl2.create_cloud(header2,fields_radar,radar_scan[:,:6])
    radar_pub.publish(radar_pcl)

def talker():

    rospy.init_node('pcl2_pub_example')
    pcl_pub = rospy.Publisher("/lidar", PointCloud2,queue_size=100)
    radar_pub = rospy.Publisher("/radar", PointCloud2,queue_size=100)
    cam_pub = rospy.Publisher("/camera", Image,queue_size=100)
    rospy.loginfo("Initializing sample pcl2 publisher node...")
    rospy.sleep(1.)

    rate = rospy.Rate(10) # 10hz

    directoryLidar = '/home/abekabe/Documents/Jerry/view_of_delft_PUBLIC/lidar/training/velodyne/'
    directoryRadar = '/home/abekabe/Documents/Jerry/view_of_delft_PUBLIC/radar/training/velodyne/'
    directoryImg = '/home/abekabe/Documents/Jerry/view_of_delft_PUBLIC/lidar/training/image_2/'
    directory_bin = os.fsencode(directoryLidar)
    
    count = 0 

    br = CvBridge()

    for file in tqdm(sorted(os.listdir(directory_bin))):
        if rospy.is_shutdown(): break
        filename = os.fsdecode(file)
        if filename.endswith(".bin"): 
            #header
            lidar_file = directoryLidar + "/" + filename
            radar_file = directoryRadar + "/" + filename

            frame_id = (filename.split('.'))[0]
            img_file = directoryImg + "/" + frame_id + '.jpg'
            
            if (os.path.isfile(img_file)):
                img = cv2.imread(img_file)
                cam_pub.publish(br.cv2_to_imgmsg(img))
            else:
                print('no frame: '+ frame_id)

            lidar_scan = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)
            radar_scan = np.fromfile(radar_file, dtype=np.float32).reshape(-1, 7)
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'lidar'
            #create pcl from points
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('i', 16, PointField.FLOAT32, 1)]
            scaled_polygon_pcl = pcl2.create_cloud(header,fields,lidar_scan[:,:])

            header2 = std_msgs.msg.Header()
            header2.stamp = rospy.Time.now()
            header2.frame_id = 'radar'

            fields_radar = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('RCS', 16, PointField.FLOAT32, 1),
                    PointField('v_r', 24, PointField.FLOAT32, 1),
                    PointField('v_r_compensated', 32, PointField.FLOAT32, 1)]
            radar_pcl = pcl2.create_cloud(header2,fields_radar,radar_scan[:,:6])

            plot_ave(radar_scan)

            pcl_pub.publish(scaled_polygon_pcl)
            radar_pub.publish(radar_pcl)
            rate.sleep()
            count+=1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass