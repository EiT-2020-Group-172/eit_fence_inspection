#!/usr/bin/env python
# license removed for brevity
import rospy
import mavros.setpoint
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from std_msgs.msg import String

class stateMachine():

	def __init__(self):

        self.closerInspection=False
        self.endOfFenceReached=False
        self.current_pose = PoseStamped()
        self.current_fence_pose = Vector3()

        # setup subscriber
        
        self._local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_position_callback)
        self._fenceDistance_sub = rospy.Subscriber('/eit/fence/pose', Vector3, self._fenceDistanceCallback) # change topic and message type
        

        # setup publisher
        self._setpoint_local_pub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped, queue_size=10)



		#https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
		
		#rospy.init_node('stateMachine', anonymous=True)
		
		
		
	def endOfFenceReached(self):
		# check if drone has reached end of fence, maybe done using a gps?
		return False

    def _local_position_callback(self, topic):
        self.current_pose = topic
        

	def _fenceDistanceCallback(self, data):

		# get fence location data, command drone to adjust position according to data, can give a drone 
		# position in local coordinates and a yaw angle. 
        # angle is zero if drone is paralel to fence
		if self.closerInspection: # if were doing a closer inspection, no need to move on
			pass
		else:
            self.current_fence_pose = data
            _adjust_drone_pos()

    def _adjust_drone_pos(self):
        setpointMsg=PoseStamped()
        self._setpoint_local_pub.publish(setpointMsg)

			

	def loop(self):
		
		if endOfFenceReached():
			endOfFenceReached = True
			#start flying home
		else:
			pass
			# keep following fence




		
	
		

if __name__ == '__main__':
    try:
        sm = stateMachine()
		sm.loop()
		

    except rospy.ROSInterruptException:
        pass