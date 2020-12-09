#!/usr/bin/env python
# license removed for brevity
# Notes: The following is true if distanceinverted is not true
# Distance too large means move to the right
# If distance too small, move left
# positve rotation is counter clockwise


import rospy
import mavros.setpoint
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from std_msgs.msg import String, Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.spatial.transform import Rotation as R
import numpy as np
from mavros_msgs.msg import State, PositionTarget
import mavros_msgs.srv
from radar_test_package.mav import Mav 
import radar_test_package.message_tools as msgt

def yaw_to_orientation(yaw):
    quat_tf = quaternion_from_euler(0, 0, yaw)
    ori = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
    return ori

def quatToRotation(quaternion):
    #print(quaternion.x)
    return R.from_quat([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
    
def angleToRotation(angle):
    return R.from_euler('z',angle)


class stateMachine():
    def __init__(self):
        #print("Init")
        rospy.init_node('stateMachine', anonymous=True)
        #self.desiredDistance=0  
        self.closerInspection = False
        self.endOfFenceReached = False
        self.breachDetected = False
        self.current_pose = PoseStamped()
        self.current_fence_pose = Vector3()
        self.UAV_state = mavros_msgs.msg.State()
        self.mav = Mav()
        self.state = "off"
        
        # load parameters:

        if rospy.has_param('/desiredDistance'):
            self.desiredDistance = rospy.get_param('/desiredDistance')
            
        else:
            print("desired distance parameter not found")
            self.desiredDistance = 0

        if rospy.has_param('/forwardSpeed'):
            self.forwardSpeed = rospy.get_param('/forwardSpeed')
        else:
            print("forward speed parameter not found")
            self.forwardSpeed = 0.5

        if rospy.has_param('/distanceInverted'):
            self.distanceInverted = rospy.get_param('/distanceInverted')
        else:
            print("distance Inverted  parameter not found")
            self.distanceInverted = False

        if rospy.has_param('/rotationInverted'):
            self.rotationInverted = rospy.get_param('/rotationInverted')
        else:
            print("rotation Inverted  parameter not found")
            self.rotationInverted = False
        # setup subscriber

        self._local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_position_callback)
        self._fenceDistance_sub = rospy.Subscriber('/fence_detector/fence_est', Vector3, self._fenceDistanceCallback) # change topic and message type
        self._state_sub = rospy.Subscriber('/mavros/state', State, self._state_callback)
        self._breach_sub = rospy.Subscriber('/breachDetected', Bool, self._breach_detected_callback)

        # setup publisher
        self._setpoint_local_pub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped, queue_size=10)
        

        # setup services:
        self.set_arming = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        self.set_mode = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)

        print("published created")


        #https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED


        


    def isEndOfendFenceReached(self):
        # check if drone has reached end of fence, maybe done using a gps?
        return False

    def _local_position_callback(self, topic):
        self.current_pose = topic

    def _breach_detected_callback(self, msg = Bool()):
        if self.breachDetected == False and  msg.data:
            self.breachPose = self.current_pose
        self.breachDetected = msg.data
        
        

    def _fenceDistanceCallback(self, data):
        #print("Got Message")
        # get fence location data, command drone to adjust position according to data, can give a drone 
        # position in local coordinates and a yaw angle. 
        # angle is zero if drone is paralel to fence
        if self.closerInspection: # if were doing a closer inspection, no need to move on
            pass
        else:
            self.current_fence_pose = data
            

    def _adjust_drone_pos(self):
        #setpointMsg=PoseStamped()
        setpointMsg=mavros.setpoint.PoseStamped(
        header=mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now()),
        )
        distanceError=self.desiredDistance - self.current_fence_pose.x

        if (self.current_fence_pose.y > 0.75):
            self.current_fence_pose.y = 0.75
        elif (self.current_fence_pose.y < -0.75):
            self.current_fence_pose.y = -0.75

        current_rot=quatToRotation(self.current_pose.pose.orientation)
        if self.rotationInverted:
            current_rot=current_rot*angleToRotation(-self.current_fence_pose.y)  # as dcm?
        else:
            current_rot=current_rot*angleToRotation(self.current_fence_pose.y)  # as dcm?
        current_rot_as_dcm=current_rot.as_dcm()
        
        #distanceToAdjust=current_rot_as_dcm*current_rot_as_dcm
        arr1 = np.array([[0, self.forwardSpeed, 0]])

        if self.distanceInverted:
            arr2 = np.array([[-distanceError, 0, 0]])
        else:
            arr2 = np.array([[distanceError, 0, 0]])
        distanceToAdjust=arr2.dot(current_rot_as_dcm)+arr1.dot(current_rot_as_dcm)
        #distanceToAdjust=current_rot_as_dcm*[0,distanceError,0]#+current_rot_as_dcm*[0.5,0,0]
        #distanceToAdjust=current_rot*[0,distanceError,0]+current_rot*[0.5,0,0]
        #print(distanceToAdjust)
        #print(current_rot_as_dcm)
        setpointMsg.pose.position.x=self.current_pose.pose.position.x+distanceToAdjust[0,0]
        setpointMsg.pose.position.y=self.current_pose.pose.position.y+distanceToAdjust[0,1]
        setpointMsg.pose.position.z=1#setpointMsg.pose.position.z+distanceToAdjust[0,2]

        current_rot_quat=current_rot.as_quat()
        setpointMsg.pose.orientation.x=current_rot_quat[0]
        setpointMsg.pose.orientation.y=current_rot_quat[1]
        setpointMsg.pose.orientation.z=current_rot_quat[2]
        setpointMsg.pose.orientation.w=current_rot_quat[3]

        #print(distanceToAdjust)
        if self.breachDetected:
            setpointMsg=self.breachPose
            #self.mav.set_target_pose(self.current_pose)
        #else:
        self.mav.set_target_pose(setpointMsg)

        #self._setpoint_local_pub.publish(setpointMsg)
    def takeoff(self):
        self.mav.set_target_pose(msgt.create_setpoint_message_xyz_yaw(0, 3, 1, yaw=-2))


    def loop(self):
        while self.isEndOfendFenceReached() is not True:
            #if self.UAV_state.mode != "OFFBOARD":
            #    self.set_mode(0, 'OFFBOARD')
            #    print("enabling offboard mode")
            #    self.last_request = rospy.Time.now()
            #if not self.UAV_state.armed:
            #    if self.set_arming(True):
            #        print("Vehicle armed")
            #    self.last_request = rospy.Time.now()
                if self.mav.UAV_state.mode != "OFFBOARD":
                    self.mav.set_mode(0, 'OFFBOARD')
                    #rospy.loginfo("enabling offboard mode")
                if not self.mav.UAV_state.armed:
                    if self.mav.set_arming(True):
                        pass
                     #   rospy.loginfo("Vehicle armed")
                elif self.state == "off":
                    self.takeoff()
                    self.state = "waiting_to_arrive"
                
                elif self.state == "waiting_to_arrive":
                    if self.mav.has_arrived():
                        self.state = "find_fence"

                elif self.state == "find_fence":
                    # to be implemented
                    self.state = "fence_following"

                elif self.state == "fence_following":
                    self._adjust_drone_pos()

                rospy.sleep(0.01)
            
        
    def _state_callback(self, topic):
        self.UAV_state.armed = topic.armed
        self.UAV_state.connected = topic.connected
        self.UAV_state.mode = topic.mode
        self.UAV_state.guided = topic.guided
        


		
	
		

if __name__ == '__main__':
    try:
        sm = stateMachine()
        #sm.__init__()
        rospy.sleep(5)
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        while (not sm.UAV_state.connected):
            rospy.sleep(0.1)
        #print("connected")
        #sm._adjust_drone_pos()
        sm.loop()
        rospy.spin()
		

    except rospy.ROSInterruptException:
        pass
