#!/usr/bin/python

import rospy
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Vector3, Point32, PoseStamped, Point, Quaternion
from gazebo_msgs.srv import GetModelState
from math import atan, cos, sin, pi, sqrt
inf = float('inf')
import argparse
import sys
from random import randint

from radar_test_package.message_tools import quaternion_from_euler, create_setpoint_message_xyz_yaw 


import json



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

        self.last_model_state = None

        self.modified_pcl_pub = rospy.Publisher(
                self.node_name + "/modified_pcl",
                PointCloud,
                queue_size=1)


        self.fence_pose_pub = rospy.Publisher(
                self.node_name + "/fence_pose",
                PoseStamped,
                queue_size=1)

        self.fence_1_pose_pub = rospy.Publisher(
                self.node_name + "/fence_1_pose",
                PoseStamped)

        self.fence_pose_unfiltered_pub = rospy.Publisher(
                self.node_name + "/fence_pose_unfiltered",
                PoseStamped)

        self.segmented_pcl_pub = rospy.Publisher(
                self.node_name + "/segmented_pcl",
                PointCloud,
                queue_size=1)

        self.far_pcl_pub = rospy.Publisher(
                self.node_name + "/far_pcl",
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

        if rospy.has_param("/simulation"):
            self.simulation = rospy.get_param("/simulation")
        else:
            self.simulation = False

        if rospy.has_param("/use_ransac"):
            self.use_ransac = rospy.get_param("/use_ransac")
        else:
            self.use_ransac = False

        if rospy.has_param("/ransac_n_points"):
            self.ransac_n_points = rospy.get_param("/ransac_n_points")
        else:
            self.ransac_n_points = 25

        if rospy.has_param("/ransac_max_it"):
            self.ransac_max_it = rospy.get_param("/ransac_max_it")
        else:
            self.ransac_max_it = 100

        if rospy.has_param("/ransac_accept_r2"):
            self.ransac_accept_r2 = rospy.get_param("/ransac_accept_r2")
        else:
            self.ransac_accept_r2 = 0.95

        if rospy.has_param("/far_points_min_dist"):
            self.far_points_min_dist = rospy.get_param("/far_points_min_dist")
        else:
            self.far_points_min_dist = 0.25

        if rospy.has_param("/corner_ang_tol"):
            self.corner_ang_tol = rospy.get_param("/corner_ang_tol")
        else:
            self.corner_ang_tol = 0.25

        if rospy.has_param("/ransac_corner_accept_r2"):
            self.ransac_corner_accept_r2 = rospy.get_param("/ransac_corner_accept_r2")
        else:
            self.ransac_corner_accept_r2 = 0.7

    def publish_fence_pose(
            self,
            dist,
            ang,
            publisher
    ):
        pose = create_setpoint_message_xyz_yaw(0, 0, dist, yaw=0)
        point = Point()
        point.z = dist
        point.x = 0
        point.y = 0
        quat = quaternion_from_euler(0, ang, 0)
        q = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose.pose.orientation = q
        pose.pose.position = point

        pose.header.frame_id = "camera_link"

        publisher.publish(pose)

    def publish_pointcloud(
            self,
            pcl,
            publisher
    ):
        points = []

        for point in pcl:
            p = Point32()
            p.x = -point[2]
            p.y = point[1]
            p.z = point[0]
            #p.x = point[0]
            #p.y = point[1]
            #p.z = point[2]

            points.append(p)

        msg = PointCloud()
        msg.points = points
        #msg.header.frame_id = "ti_mmwave_pcl"
        msg.header.frame_id = "camera_link"

        publisher.publish(msg)

    def on_new_msg(
            self,
            msg
    ):
        pcl = pointcloud2_to_xyz_array(msg)

        if self.simulation:
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

        #points = []

        #for point in self.fence_pcl:
        #    p = Point32()
        #    p.x = -point[2]
        #    p.y = point[1]
        #    p.z = point[0]

        #    points.append(p)

        #msg = PointCloud()
        #msg.points = points
        #msg.header.frame_id = "camera_link"

        #self.segmented_pcl_pub.publish(msg)

        self.publish_pointcloud(
                self.fence_pcl,
                self.segmented_pcl_pub
        )
        
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

            self.publish_fence_pose(
                    dist_out,
                    ang_out,
                    self.fence_pose_pub
            )

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
            #p.x = z 
            #p.y = y 
            #p.z = -x 
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
        X_flat = X[:,0].flatten()

        r2 = self.calc_r2(
                X_flat, 
                Y_flat, 
                beta[0], 
                beta[1]
        )

        return beta[0], beta[1], r2

    def calc_r2(
            self,
            X,
            Y,
            a,
            b
    ):
        y_bar = np.mean(Y)

        SST = np.sum(
                np.power(
                    np.subtract(
                        Y,
                        y_bar
                    ),
                    2
                )
        )

        Y_hat = np.add(
                np.multiply(
                    X,
                    a
                ),
                b
        )

        SSRes = np.sum(
                np.power(
                    np.subtract(
                        Y,
                        Y_hat
                    ),
                    2
                )
        )

        r2 = 1 - (SSRes / SST)

        return r2

    def ransac_fence(
            self,
            fence_pcl
    ):
        best_r2 = -inf
        best_a = None
        best_b = None

        for i in range(self.ransac_max_it):
            rand_indices = np.random.choice(
                    fence_pcl.shape[0],
                    size=self.ransac_n_points,
                    replace=False
            )
            
            points = fence_pcl[rand_indices,:]

            a, b, r2 = self.fit_line(points)

            if r2 > best_r2:
                best_r2 = r2
                best_a = a
                best_b = b

                #if r2 > self.ransac_accept_r2:
                #    break

        return best_a, best_b, best_r2

    #def ransac_corner(
    #        self,
    #        fence_pcl
    #):
    #    best_a_1, best_b_1, best_a_2, best_b_2 = None, None, None, None
    #    best_r2_1, best_r2_2 = -inf, -inf

    #    for i in range(self.ransac_max_it):
    #        rand_indices = np.random.choice(
    #                fence_pcl.shape[0],
    #                size=self.ransac_n_points,
    #                replace=False
    #        )
    #        
    #        points_1 = fence_pcl[rand_indices,:]

    #        a_1, b_1, r2_1 = self.fit_line(points_1)

    #        far_points = self.get_far_points(
    #                fence_pcl,
    #                a_1,
    #                b_1
    #        )

    #        #self.publish_pointcloud(
    #        #        far_points,
    #        #        self.far_pcl_pub
    #        #)


    #        if far_points.shape[0] < self.ransac_n_points * 2:
    #            continue

    #        rand_indices_2 = np.random.choice(
    #                far_points.shape[0],
    #                #fence_pcl.shape[0],
    #                size=self.ransac_n_points,
    #                replace=False
    #        )
    #        
    #        points_2 = far_points[rand_indices_2,:]
    #        #points_2 = fence_pcl[rand_indices_2,:]

    #        a_2, b_2, r2_2 = self.fit_line(points_2)

    #        ang_1 = atan(a_1)
    #        ang_2 = atan(a_2)
    #        ang_diff = abs(ang_1 - ang_2)

    #        if r2_1 > best_r2_1 and r2_2 > best_r2_2 and abs(ang_diff - pi/2) <= self.corner_ang_tol:
    #            #print("ang1: " + str(ang_1) + "\tangf2: " + str(ang_2) + "\n\n")
    #            best_a_1, best_b_1, best_a_2, best_b_2 = a_1, b_1, a_2, b_2
    #            best_r2_1, best_r2_2 = r2_1, r2_2

    #            #if best_r2_1 >= self.ransac_corner_accept_r2 and best_r2_2 >= self.ransac_corner_accept_r2:
    #            #    break

    #    return best_a_1, best_b_1, best_r2_1, best_a_2, best_b_2, best_r2_2

    def get_far_points(
            self,
            fence_pcl,
            a,
            b
    ):
        closest_points_on_line_X = np.divide(
                np.subtract(
                    np.add(
                        fence_pcl[:,2],
                        a * fence_pcl[:,0]
                    ),
                    a * b
                ),
                a**2 + 1
        )

        closest_points_on_line_Y = np.add(
                np.multiply(
                    closest_points_on_line_X,
                    a
                ),
                b
        )

        dist = np.sqrt(
                np.add(
                    np.square(
                        np.subtract(
                            closest_points_on_line_X,
                            fence_pcl[:,2]
                        )
                    ),
                    np.square(
                        np.subtract(
                            closest_points_on_line_Y,
                            fence_pcl[:,0]
                        )
                    )
                )
        )

        return fence_pcl[dist >= self.far_points_min_dist]

    def dist_ang_fence(
            self,
            a,
            b
    ):
        closest_x = (- a * b) / (a**2 + 1)
        closest_y = a * closest_x + b
        dist = sqrt(closest_x**2 + closest_y**2)
        ang = atan(a)

        return dist, ang

    def dist_ang_corner(
            self,
            a_1,
            b_1,
            a_2,
            b_2
    ):
        dist, ang = self.dist_ang_fence(a_1, b_1)
        dist_c, ang_c = self.dist_ang_fence(a_2, b_2)

        #if ang < 0:
        #    ang = 2 * pi - ang
        #if ang_c < 0:
        #    ang_c = 2 * pi - ang_c

        ang_ret = (ang + ang_c) / 2
        #dist_ret = dist_c
        #if (dist > dist_c):
        #    dist_ret = dist

        x_intersect

        return (dist + dist_c) / 2, (ang + ang_c) / 2, dist_c, ang_c

    def get_fence_pos(
            self,
            fence_pcl
    ):
        dist = None
        ang = None

        if fence_pcl.shape[0] >= self.min_fence_points:
            a, b, r2 = None, None, None

            if self.use_ransac:
                a, b, r2 = self.ransac_fence(fence_pcl)

                #far_points = self.get_far_points(
                #        fence_pcl,
                #        a,
                #        b
                #)

                #if far_points.shape[0] > self.ransac_n_points * 2:  # Possibly second fence
                #    a_1, b_1, r2_1, a_2, b_2, r2_2 = self.ransac_corner(fence_pcl)

                #    if r2_1 >= self.ransac_corner_accept_r2 and r2_2 >= self.ransac_corner_accept_r2:    #   Fence corner found
                #        dist, ang, dist_c, ang_c = self.dist_ang_corner(
                #                a_1,
                #                b_1,
                #                a_2,
                #                b_2
                #        )

                #        self.publish_fence_pose(
                #                dist_c,
                #                ang_c,
                #                self.fence_1_pose_pub)
                #    elif r2 >= self.ransac_accept_r2:
                if r2 >= self.min_r2_fence:
                    dist, ang = self.dist_ang_fence(
                            a,
                            b
                    )
                #elif r2 >= self.ransac_accept_r2:
                #    dist, ang = self.dist_ang_fence(
                #            a,
                #            b
                #    )
            else:
                a, b, r2 = self.fit_line(fence_pcl)

                if r2 >= self.min_r2_fence:
                    dist, ang = self.dist_ang_fence(
                            a,
                            b
                    )

        if(dist is not None and ang is not None):
            self.publish_fence_pose(
                    dist,
                    ang,
                    self.fence_pose_unfiltered_pub
            )

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
