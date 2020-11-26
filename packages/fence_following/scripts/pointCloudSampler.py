#!/usr/bin/python

import rospy
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_array, array_to_pointcloud2
from sensor_msgs.msg import PointCloud2
import argparse

class PointCloudSampler:
    def __init__(
            self,
            node_name="/point_cloud_sampler",
            in_topic="/ti_mmwave/radar_scan_pcl",
            out_topic="sampled_pcl",
            sample_n=5
    ):
        self.node_name = node_name
        self.in_topic = in_topic   
        self.out_topic = out_topic
        self.sample_n = sample_n

        self.point_cloud_buffer = []
        self.buffer_pnt = -1

        rospy.init_node(self.node_name)

        self.pub = rospy.Publisher(
                self.node_name + self.out_topic,
                PointCloud2,
                queue_size=10
        )

        rospy.Subscriber(
                self.in_topic,
                PointCloud2,
                self.on_new_msg
        )

    def on_new_msg(
            self,
            msg
    ):
        if (len(self.point_cloud_buffer) < self.sample_n):
            self.point_cloud_buffer.append(pointcloud2_to_array(msg))

        else:
            self.point_cloud_buffer[self.buffer_pnt] = pointcloud2_to_array(msg)

        self.buffer_pnt += 1
        self.buffer_pnt %= self.sample_n

        if (len(self.point_cloud_buffer) == self.sample_n):
            sampled_pcl = np.concatenate(tuple(self.point_cloud_buffer))

            self.pub.publish(array_to_pointcloud2(sampled_pcl))

def parse_args():
    parser = argparse.ArgumentParser(description="Samples PointCloud2 msgs into combined msgs.")

    parser.add_argument(
            "-n",
            metavar="N",
            type=str,
            dest="N",
            action="store",
            default="point_cloud_sampler",
            help="The ROS node name"
    )

    parser.add_argument(
            "-o",
            metavar="O",
            type=str,
            dest="O",
            action="store",
            default="/sampled_pcl",
            help="The ROS publish output topic"
    )

    parser.add_argument(
            "-i",
            metavar="I",
            type=str,
            dest="I",
            action="store",
            default="/ti_mmwave/radar_scan_pcl",
            help="The ROS subscribe input topic"
    )

    parser.add_argument(
            "--samples",
            metavar="samples",
            type=int,
            dest="samples",
            action="store",
            default=5,
            help="The number of sampled messages to combine into one message"
    )

    args = parser.parse_args()

    return args

if __name__ == "__main__":
    args = parse_args()

    pcl_sampler = PointCloudSampler(
            node_name=args.N,
            in_topic=args.I,
            out_topic=args.O,
            sample_n=args.samples
    )

    rospy.spin()
