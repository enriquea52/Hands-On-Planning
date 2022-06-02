#!/usr/bin/python3

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg


rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

pc_pub = rospy.Publisher("~converted_pc", PointCloud2, queue_size=1)

def scan_cb(msg):

    pc2_msg = lp.projectLaser(msg)
    pc_pub.publish(pc2_msg)


print("LASER SCAN TO POINT CLOUD CONVERTER NODE IGNITED")

scan_topic = rospy.get_param("/tbot_rplidar_scan_topic")

rospy.Subscriber(scan_topic, LaserScan, scan_cb, queue_size=1)

rospy.spin()