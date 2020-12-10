#!/usr/bin/python

import rospy
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Vector3, Point32, PoseStamped, Point, Quaternion
from math import atan, cos, sin
import argparse
import sys
from random import randint

from radar_test_package.message_tools import quaternion_from_euler, create_setpoint_message_xyz_yaw 

class FenceDetector:
    def __init__(
            self,
            node_name="fence_detector",
            in_topic="/point_cloud_sampler/sampled_pcl",
            out_topic="/fence_est",
            init_dist=2,
            init_ang=0,
            min_dist_x=1.1,
            max_dist_x=6,
            height_thresh=-0.7
    ):
        self.node_name = node_name
        self.in_topic = in_topic
        self.out_topic = out_topic

        if rospy.has_param("/init_dist"):
            self.dist = rospy.get_param("/init_dist")
        else:
            self.dist = init_dist

        if rospy.has_param("/init_ang"):
            self.ang = rospy.get_param("/init_ang")
        else:
            self.ang = init_ang

        if rospy.has_param("/min_dist_x"):
            self.min_dist_x = rospy.get_param("/min_dist_x")
        else:
            self.min_dist_x = min_dist_x

        if rospy.has_param("/max_dist_x"):
            self.max_dist_x = rospy.get_param("/max_dist_x")
        else:
            self.max_dist_x = max_dist_x

        if rospy.has_param("/height_thresh"):
            self.height_thresh = rospy.get_param("/height_thresh")
        else:
            self.height_thresh = height_thresh

        rospy.init_node(self.node_name)

        self.rate = rospy.Rate(1000)

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

        self.modified_pcl_pub = rospy.Publisher(
                self.node_name + "/modified_pcl",
                PointCloud,
                queue_size=1)


        self.fence_pose_pub = rospy.Publisher(
                self.node_name + "/fence_pose",
                PoseStamped,
                queue_size=1)

        self.segmented_pcl_pub = rospy.Publisher(
                self.node_name + "/segmented_pcl",
                PointCloud,
                queue_size=1)

        self.pcl = None
        self.pcl_filtered = None
        self.pcl_fence = None

        self.last_dist_out = 0
        self.last_ang_out = 0
        if rospy.has_param("/lp_alpha_dist"):
            self.alpha_dist = rospy.get_param("/lp_alpha_dist")
        else:
            self.alpha_dist = 0.1

        if rospy.has_param("/lp_alpha_ang"):
            self.alpha_ang = rospy.get_param("/lp_alpha_ang")
        else:
            self.alpha_ang = 0.1

        if rospy.has_param("/min_fence_points"):
            self.min_fence_points = rospy.get_param("/min_fence_points")
        else:
            self.min_fence_points = 5

        if rospy.has_param("/min_r2_fence"):
            self.min_r2_fence = rospy.get_param("/min_r2_fence")
        else:
            self.min_r2_fence = 0.8

    def on_new_msg(
            self,
            msg
    ):
        pcl = pointcloud2_to_xyz_array(msg)

        pcl = self.transform_pcl_coordsys(pcl)

        self.pcl = pcl

        try:
            pcl = self.remove_close_points(pcl)
        except(IndexError):
            return

        self.pcl_filtered = pcl

        fence_pcl = self.segment_fence_pcl(pcl)

        if (fence_pcl is None):
            return

        self.fence_pcl = fence_pcl

        points = []

        for point in self.fence_pcl:
            p = Point32()
            p.x = -point[2]
            p.y = point[1]
            p.z = point[0]

            points.append(p)

        msg = PointCloud()
        msg.points = points
        msg.header.frame_id = "camera_link"

        self.segmented_pcl_pub.publish(msg)
        
        dist, ang = self.get_fence_pos(
                self.fence_pcl
        )

        if (dist is not None):
            self.dist = dist
        if (ang is not None):
            self.ang = ang

        if (dist is not None and ang is not None):
            dist_out = self.last_dist_out + self.alpha_dist * (dist - self.last_dist_out)
            ang_out = self.last_ang_out + self.alpha_ang * (ang - self.last_ang_out)
            self.last_dist_out = dist_out
            self.last_ang_out = ang_out

            msg = Vector3()
            msg.x = dist_out
            msg.y = ang_out
            msg.z = 0 

            self.pub.publish(msg)
        
            pose = create_setpoint_message_xyz_yaw(0, 0, dist_out, yaw=0)
            point = Point()
            point.z = dist_out
            point.x = 0
            point.y = 0
            quat = quaternion_from_euler(0, ang_out, 0)
            q = Quaternion(quat[0], quat[1], quat[2], quat[3])
            pose.pose.orientation = q
            pose.pose.position = point

            pose.header.frame_id = "camera_link"

            self.fence_pose_pub.publish(pose)

    def transform_pcl_coordsys(self, points):
        def rm_pnt(dist):
            a = 0.5
            pct = 100 - 100 * a**dist
            
            n = randint(0, 100)

            if (pct > n):
                return True
            else:
                return False

        points_arr = []
        points_msg = []
        for point in points:
            x, y, z = (point[0], point[1], point[2])

            if rm_pnt(z):
                #del point
                continue

            points_arr.append([z, y, -x])

            p = Point32()
            p.x = x 
            p.y = y 
            p.z = z 
            points_msg.append(p)

        msg = PointCloud()
        msg.points = points_msg
        msg.header.frame_id = "camera_link"

        self.modified_pcl_pub.publish(msg)

        return np.array(points_arr)

    def start(self):
        self.plot_count = 0

        while(True):
            self.rate.sleep()

            if (rospy.is_shutdown()):
                break

    def remove_close_points(
            self,
            point_cloud
    ):
        return point_cloud[(point_cloud[:,0] >= self.min_dist_x) & (point_cloud[:,0] <= self.max_dist_x)]

    def segment_fence_pcl(
            self,
            point_cloud
    ):
        # remove points below a certain height threshold from the camera
        point_cloud = point_cloud[(point_cloud[:,1] < (self.height_thresh))]

        if (point_cloud.shape[0] < 2):
            return None

        mean_x = np.mean(point_cloud[:,0])
        stdev_x = np.std(point_cloud[:,0])

        fence_pcl = point_cloud[
                (point_cloud[:,0] >= mean_x - stdev_x) & 
                (point_cloud[:,0] <= mean_x + stdev_x)]

        return fence_pcl

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

        Y_flat = Y.flatten()
        y_bar = np.mean(Y_flat)

        SST = np.sum(
                np.power(
                    np.subtract(
                        Y_flat,
                        y_bar
                    ),
                    2
                )
        )

        X_flat = X[:,0].flatten()

        Y_hat = np.add(
                np.multiply(
                    X_flat,
                    beta[0]
                ),
                beta[1]
        )

        SSRes = np.sum(
                np.power(
                    np.subtract(
                        Y_flat,
                        Y_hat
                    ),
                    2
                )
        )

        r2 = 1 - (SSRes / SST)

        return beta[0], beta[1], r2

    def get_fence_pos(
            self,
            fence_pcl
    ):
        dist = None
        ang = None

        if fence_pcl.shape[0] >= self.min_fence_points:
            a, b, r2 = self.fit_line(fence_pcl)
            #print(r2)
            #if r2 >= self.min_r2_fence:
            dist = b
            ang = atan(a)

        return dist, ang

    #def visualize_segmentation(
    #        self,
    #        pcl, 
    #        pcl_fence, 
    #        pcl_ground,
    #        show_plot=False,
    #        filename=None
    #):
    #    plt.Figure()

    #    plt.scatter(pcl[:,0],pcl[:,2],color='green',label="Removed points")
    #    if pcl_fence is not None:
    #        plt.scatter(pcl_fence[:,0],pcl_fence[:,2],color='red',label="Fence points")
    #    if pcl_ground is not None:
    #        plt.scatter(pcl_ground[:,0],pcl_ground[:,2],color='black',label="Ground points")

    #    plt.title("Segmented point cloud")
    #    plt.legend()
    #    plt.xlabel("Depth")
    #    plt.ylabel("Height")

    #    if filename is not None:
    #        plt.savefig(filename)

    #    if show_plot:
    #        plt.show()

    #def visualize_detection(
    #        self,
    #        pcl, 
    #        dist,
    #        ang,
    #        show_plot=False,
    #        filename=None
    #):
    #    fig = plt.Figure()

    #    plt.scatter(pcl[:,2],pcl[:,0],color='blue', label="Point cloud")
    #    dist_points = None
    #    ang_points = None

    #    if dist is not None:
    #        dist_points = [[0,0,0],[dist,0,0]] 
    #        plt.plot([dist_points[0][2],dist_points[1][2]],[dist_points[0][0],dist_points[1][0]],color="red",linestyle="--",label="Detected distance to fence")

    #        if ang is not None:
    #            ang_points = [[dist - sin(ang), 0, -cos(ang)], [dist + sin(ang), 0, cos(ang)]]
    #            plt.plot([ang_points[0][2],ang_points[1][2]],[ang_points[0][0],ang_points[1][0]],color="red",linestyle="-",label="Detected fence angle")

    #    plt.legend()
    #    plt.title("Detected fence, top view")
    #    plt.xlabel("Width")
    #    plt.ylabel("Depth")

    #    if filename is not None:
    #        plt.savefig(filename)

    #    if show_plot:
    #        plt.show()

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

    sys.argv = rospy.myargv(argv=sys.argv)

    args = parser.parse_args()

    return args

if __name__ == "__main__":
    args = parse_args()

    fd = FenceDetector(
            node_name=args.n,
            in_topic=args.i,
            out_topic=args.o,
            init_dist=args.dist,
            init_ang=args.ang
    )

    fd.start()
