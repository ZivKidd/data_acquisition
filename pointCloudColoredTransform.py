# coding:utf-8
import os
import cv2
import numpy as np
import yaml
import roslib
import message_filters
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from cv_bridge import CvBridgeError
import os
import sys
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import datetime
import struct
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
from scipy.spatial.transform import Rotation


class PointCloudColoredTransform:
    def __init__(self):
        self.bridge = CvBridge()
        self.pc_pub = rospy.Publisher('/pandar_points_colored_tranform', PointCloud2, queue_size=10)
        odom_sub = message_filters.Subscriber('/odom', Odometry)
        pc_sub = message_filters.Subscriber('/pandar_points_colored', PointCloud2)
        ts = message_filters.ApproximateTimeSynchronizer([odom_sub, pc_sub], 10,
                                                         5, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, odom, pc):
        points = point_cloud2.read_points(pc)
        points = np.array(list(points))

        points_xyz = points[:, :3]
        ones = np.ones([len(points_xyz), 1])
        points_xyz = np.concatenate([points_xyz, ones], axis=1)

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w
        R = Rotation.from_quat([qx, qy, qz, qw]).as_dcm()
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z

        points_xyz = points_xyz.dot(T)

        points_xyz[:, 3:4] = points[:, 3:4]

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]

        pc2 = point_cloud2.create_cloud(pc.header, fields, points_xyz)

        print pc.header

        self.pc_pub.publish(pc2)


rospy.init_node('pointCloudColoredTransform', anonymous=True)
pointCloudColoredTransform = PointCloudColoredTransform()
rospy.spin()
