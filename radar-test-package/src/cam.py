#!/usr/bin/env python
from math import (degrees, radians, floor, isinf, sin, cos)
import numpy as np

import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import LaserScan, Range, Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Vector3
from radar_test_package.message_tools import create_setpoint_message_xyz_yaw
from ros_numpy.point_cloud2 import pointcloud2_to_array, array_to_pointcloud2

depth_image_topic = '/camera/depth/image_raw'
rgb_image_topic = '/camera/rgb/image_raw'
cloud_topic = '/camera/depth/points'
pose_est_topic = '/fence_detector/fence_est'
mmwave_topic = '/ti_mmwave/radar_scan_pcl'


class LidarProcessor():
    def __init__(self):
        rospy.init_node('depth_image_processing')

        # self._depth_image_sub = rospy.Subscriber(depth_image_topic, Image, self.on_depth_image)
        self._cloud_sub = rospy.Subscriber(cloud_topic, PointCloud2, self.on_cloud)
        self.cloudcounter = 0
        self._fence_sub = rospy.Subscriber(pose_est_topic, Vector3, self.on_pose_est)


        # self._displacement_pub = rospy.Publisher(topic_displacement, PoseStamped, queue_size=10)
        self._transformed_pcl_pub = rospy.Publisher(mmwave_topic, PointCloud2,  queue_size=10)

        self.bridge = CvBridge()
    
    def on_depth_image(self, message = Image()):
        cv_image = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
        # print(cv_image.shape)
        subimg = self.crop_img(cv_image)
        # print(subimg.shape)
        result = np.min(subimg)
        print(str(result))
    
    def crop_img(self, img):
        w = img.shape[0]
        h = img.shape[1]
        return img[w/3:2*w/3,h/3:2*h/3]

    def on_cloud(self, msg = PointCloud2()):
        self.cloudcounter = self.cloudcounter + 1
        if self.cloudcounter % 1000 == 0:
            self.cloudcounter = 0
            rospy.logout('received 1000 clouds')

        # arr = pointcloud2_to_array(msg)
        # # arr contains 60 rows
        # for row in arr:
        #     for index, point in enumerate(row):
        #         x, y, z, r = point
        #         rospy.logout(point)
        #         point = (z, x, y, r)
        #         # row[index] = point
        #         rospy.logout(point)
        #         return
        # msg = array_to_pointcloud2(arr)
        self._transformed_pcl_pub.publish(msg)

    def on_pose_est(self, msg = Vector3):
        rospy.logout(msg)
        pass


if __name__ == "__main__":
    ls = LidarProcessor()
    rospy.spin()
