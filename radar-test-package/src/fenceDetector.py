#!/usr/bin/python

import rosbag
import rospy
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3
from math import atan, cos, sin
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from threading import Lock
import argparse
import sys

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

MIN_DIST_X = 1.1
MAX_DIST_X = 5

class FenceDetector:
    def __init__(
            self,
            node_name="fence_detector",
            in_topic="/point_cloud_sampler/sampled_pcl",
            out_topic="/fence_est",
            kf_sample_time_ms=100,
            init_dist=2,
            init_height=1,
            init_ang=0
    ):
        self.node_name = node_name
        self.in_topic = in_topic
        self.out_topic = out_topic

        self.kf_dist = KalmanFilter(dim_x=2, dim_z=1)
        self.kf_dist.x = np.array([[float(init_dist)],[0.]])
        self.kf_dist.F = np.array([[1., float(kf_sample_time_ms)],[0., 1.]])
        self.kf_dist.H = np.array([[1., 0.]])
        self.kf_dist.P *= 1000
        self.kf_dist.R = 5.
        self.kf_dist.Q = Q_discrete_white_noise(dim=2, dt=kf_sample_time_ms, var=0.13)
        self.dist_lock = Lock()

        self.kf_ang = KalmanFilter(dim_x=2, dim_z=1)
        self.kf_ang.x = np.array([[float(init_ang)],[0.]])
        self.kf_ang.F = np.array([[1., float(kf_sample_time_ms)],[0., 1.]])
        self.kf_ang.H = np.array([[1., 0.]])
        self.kf_ang.P *= 1000
        self.kf_ang.R = 5.
        self.kf_ang.Q = Q_discrete_white_noise(dim=2, dt=kf_sample_time_ms, var=0.13)
        self.ang_lock = Lock()

        self.kf_height = KalmanFilter(dim_x=2, dim_z=1)
        self.kf_height.x = np.array([[float(init_height)],[0.]])
        self.kf_height.F = np.array([[1., 0.],[0., 0.]])
        self.kf_height.H = np.array([[1., 0.]])
        self.kf_height.P *= 1000
        self.kf_height.R = 5.
        self.kf_height.Q = Q_discrete_white_noise(dim=2, dt=kf_sample_time_ms, var=0.13)
        #self.height = init_height
        self.height_lock = Lock()

        rospy.init_node(self.node_name)

        self.rate = rospy.Rate(1000/kf_sample_time_ms)

        self.pub = rospy.Publisher(
                self.node_name + self.out_topic,
                Vector3,
                queue_size=1
        )

        rospy.Subscriber(
                self.in_topic,
                PointCloud2,
                self.on_new_msg
        )

        self.pcl = None
        self.pcl_filtered = None
        self.pcl_fence = None
        self.pcl_ground = None

    def on_new_msg(
            self,
            msg
    ):
        pcl = pointcloud2_to_xyz_array(msg)
        rospy.loginfo(pcl.shape)

        self.transform_pcl_coordsys(pcl)

        self.pcl = pcl

        pcl = self.remove_close_points(pcl)

        self.pcl_filtered = pcl

        self.fence_pcl, self.ground_pcl = self.segment_fence_pcl(pcl)
        
        dist, ang, height = self.get_fence_pos(
                self.fence_pcl,
                self.ground_pcl
        )

        if dist is not None:
            self.dist_lock.acquire(True)
            self.kf_dist.update(np.array([dist]))
            self.dist_lock.release()

        if ang is not None:
            self.ang_lock.acquire(True)
            self.kf_ang.update(np.array([ang]))
            self.ang_lock.release()

        if height is not None:
            self.height_lock.acquire(True)
            self.kf_height.update(np.array([height]))
            #self.height = height
            self.height_lock.release()
    
    def transform_pcl_coordsys(self, points):
        for point in points:
            x, y, z = (point[0], point[1], point[2])
            point[0] = z
            point[1] = x
            point[2] = y

    def start(self):
        self.plot_count = 0

        while(True):
            self.rate.sleep()

            self.dist_lock.acquire(True)
            self.kf_dist.predict()
            dist = np.copy(self.kf_dist.x)[0]
            self.dist_lock.release()

            self.ang_lock.acquire(True)
            self.kf_ang.predict()
            ang = np.copy(self.kf_ang.x)[0]
            self.ang_lock.release()

            self.height_lock.acquire(True)
            self.kf_height.predict()
            height = np.copy(self.kf_height.x)[0]
            #height = self.height
            self.height_lock.release()
            
            msg = Vector3()
            msg.x = dist
            msg.y = ang
            msg.z = height

            self.pub.publish(msg)

            if self.pcl is not None:
                #self.visualize_detection(
                #        self.pcl,
                #        dist,
                #        ang,
                #        filename="/home/balint/" + str(self.plot_count) + ".png"
                #)
                self.visualize_segmentation(
                        self.pcl_filtered,
                        self.pcl_fence,
                        self.pcl_ground,
                        filename="/home/balint/" + str(self.plot_count) + ".png")
                plt.clf()
                self.plot_count += 1

            if (rospy.is_shutdown()):
                break

    def remove_close_points(
            self,
            point_cloud
    ):
        return point_cloud[(point_cloud[:,0] >= MIN_DIST_X) & (point_cloud[:,0] <= MAX_DIST_X)]

    def segment_fence_pcl(
            self,
            point_cloud
    ):
        mean_x = np.mean(point_cloud[:,0])
        stdev_x = np.std(point_cloud[:,0])

        fence_pcl = point_cloud[
                (point_cloud[:,0] >= mean_x - stdev_x) & 
                (point_cloud[:,0] <= mean_x + stdev_x)]
        ground_pcl = point_cloud[
                (point_cloud[:,0] < mean_x - stdev_x) & 
                (point_cloud[:,1] < 0) &
                (point_cloud[:,1] < np.amin(fence_pcl[:,1]))]

        return fence_pcl, ground_pcl

    def fit_line(
            self,
            fence_pcl
    ):
        X = np.ndarray((fence_pcl.shape[0],2))
        X[:,0] = fence_pcl[:,2]
        X[:,1] = np.full(X[:,1].shape, 1)

        Y = fence_pcl[:,0]

        beta = np.matmul(
                np.matmul(
                    np.linalg.inv(
                        np.matmul(
                            np.transpose(X),
                            X
                        )
                    ),
                    np.transpose(X)
                ),
                Y
        )

        return beta[0], beta[1]

    def get_fence_pos(
            self,
            fence_pcl, 
            ground_pcl
    ):
        dist = None
        ang = None
        height = None

        if (fence_pcl.shape[0] == 1):
            dist = fence_pcl[0,0]
        elif (fence_pcl.shape[0] == 2):
            dist = np.mean(fence_pcl[:,0])
        elif (fence_pcl.shape[0] > 2):
            a, b = self.fit_line(fence_pcl)
            
            dist = b
            ang = atan(a)

        if (ground_pcl.shape[0] > 0):
            height = abs(np.amin(ground_pcl[:,1]))

        return dist, ang, height

    def visualize_segmentation(
            self,
            pcl, 
            pcl_fence, 
            pcl_ground,
            show_plot=False,
            filename=None
    ):
        plt.Figure()

        plt.scatter(pcl[:,0],pcl[:,2],color='green',label="Removed points")
        if pcl_fence is not None:
            plt.scatter(pcl_fence[:,0],pcl_fence[:,2],color='red',label="Fence points")
        if pcl_ground is not None:
            plt.scatter(pcl_ground[:,0],pcl_ground[:,2],color='black',label="Ground points")

        plt.title("Segmented point cloud")
        plt.legend()
        plt.xlabel("Depth")
        plt.ylabel("Height")

        if filename is not None:
            plt.savefig(filename)

        if show_plot:
            plt.show()

    def visualize_detection(
            self,
            pcl, 
            dist,
            ang,
            show_plot=False,
            filename=None
    ):
        fig = plt.Figure()

        plt.scatter(pcl[:,2],pcl[:,0],color='blue', label="Point cloud")
        dist_points = None
        ang_points = None

        if dist is not None:
            dist_points = [[0,0,0],[dist,0,0]] 
            plt.plot([dist_points[0][2],dist_points[1][2]],[dist_points[0][0],dist_points[1][0]],color="red",linestyle="--",label="Detected distance to fence")

            if ang is not None:
                ang_points = [[dist - sin(ang), 0, -cos(ang)], [dist + sin(ang), 0, cos(ang)]]
                plt.plot([ang_points[0][2],ang_points[1][2]],[ang_points[0][0],ang_points[1][0]],color="red",linestyle="-",label="Detected fence angle")

        plt.legend()
        plt.title("Detected fence, top view")
        plt.xlabel("Width")
        plt.ylabel("Depth")

        if filename is not None:
            plt.savefig(filename)

        if show_plot:
            plt.show()

def parse_args():
    parser = argparse.ArgumentParser(description="Detects the height of the drone and the distance and angle to a fence from mmWave point clouds.")

    parser.add_argument(
            "-n",
            metavar="N",
            type=str,
            dest="n",
            action="store",
            default="fence_detector",
            help="The ROS node name"
    )

    parser.add_argument(
            "-o",
            metavar="O",
            type=str,
            dest="o",
            action="store",
            default="/fence_est",
            help="The ROS publish output topic"
    )

    parser.add_argument(
            "-i",
            metavar="I",
            type=str,
            dest="i",
            action="store",
            default="/point_cloud_sampler/sampled_pcl",
            help="The ROS subscribe input topic"
    )

    parser.add_argument(
            "--kf-dt-ms",
            metavar="Dt",
            type=int,
            dest="dt",
            action="store",
            default=100,
            help="The Kalman Filter prediction time step in ms"
    )

    parser.add_argument(
            "--init-dist",
            metavar="DIST",
            type=int,
            dest="dist",
            action="store",
            default=2,
            help="The initial estimated distance to the fence in meters"
    )

    parser.add_argument(
            "--init-ang",
            metavar="ANG",
            type=int,
            dest="ang",
            action="store",
            default=0,
            help="The initial estimated angle of the fence in radians"
    )

    parser.add_argument(
            "--init-height",
            metavar="HEIGHT",
            type=int,
            dest="height",
            action="store",
            default=1,
            help="The initial estimated height of the drone from the bottom of the fence in meters"
    )

    sys.argv = rospy.myargv(argv=sys.argv)

    args = parser.parse_args()

    return args

if __name__ == "__main__":
    args = parse_args()

    fd = FenceDetector(
            node_name=args.n,
            in_topic=args.i,
            out_topic=args.o,
            kf_sample_time_ms=args.dt,
            init_dist=args.dist,
            init_ang=args.ang,
            init_height=args.height
    )

    fd.start()
