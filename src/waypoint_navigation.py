#!/usr/bin/env python

import rospy
from tum_ardrone.msg import filter_state
from geometry_msgs.msg import Twist,Vector3
from std_msgs.msg import Empty
from PIDAutoTuning import PIDAutoTuning

class Controller:

	def __init__(self):
		
		self.pitch_pid=PIDAutoTuning(0.025,0.007,0.5,0.31,True) #0.025 0.007 with error limit 0.1
		self.roll_pid=PIDAutoTuning(0.025,0.007,0.5,0.31,True)
		self.alt_pid=PIDAutoTuning(0.03,0.005,0.6,0.1,True)
		self.now=rospy.get_rostime().to_sec()
		self.pubTakeoff=rospy.Publisher("/ardrone/takeoff",Empty,queue_size=1)
		self.pubLand=rospy.Publisher("/ardrone/land",Empty,queue_size=1)
		self.pubcmd=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
		self.pub_pitch=rospy.Publisher("/ardrone/pitch_pid",Vector3,queue_size=1)
		self.pub_roll=rospy.Publisher("/ardrone/roll_pid",Vector3,queue_size=1)
		self.pub_alt=rospy.Publisher("/ardrone/alt_pid",Vector3,queue_size=1)
		self.pub_error=rospy.Publisher("/ardrone/error",Vector3,queue_size=1)
		self.pub_target=rospy.Publisher("/ardrone/target",Vector3,queue_size=1)
		self.target_count=0

	def PublishParms(self):
		self.pub_pitch.publish(Vector3(self.pitch_pid.kp,0,self.pitch_pid.kd))
		self.pub_roll.publish(Vector3(self.roll_pid.kp,0,self.roll_pid.kd))
		self.pub_alt.publish(Vector3(self.alt_pid.kp,0,self.alt_pid.kd))

	def DroneModel(self,rect_target):
		count=0 # change this for multiple loops over the same path
		while count<1: 
			pose=rospy.wait_for_message("/ardrone/predictedPose",filter_state)
			target=rect_target[self.target_count]
			self.pub_target.publish(Vector3(target[0],target[1],target[2]))
			self.pub_error.publish(Vector3(pose.y-target[0],pose.x-target[1],pose.z-target[2]))
			dt=rospy.get_rostime().to_sec()-self.now
			pitch_input=self.pitch_pid.PIDController(pose.y,target[0],dt)
			roll_input=self.roll_pid.PIDController(pose.x,target[1],dt)
			alt_input=self.alt_pid.PIDController(pose.z,target[2],dt)		
			self.pubcmd.publish(Twist(Vector3(pitch_input,-roll_input,alt_input),Vector3(0,0,0)))
			self.now=rospy.get_rostime().to_sec()
			if abs(self.pitch_pid.error)<0.1 and abs(self.roll_pid.error)<0.1 and abs(self.alt_pid.error)<0.1:
				self.target_count+=1
				print "Tuned parameters kp kd for pitch",self.pitch_pid.kp,self.pitch_pid.kd
				print "Tuned parameters kp kd for roll",self.roll_pid.kp,self.roll_pid.kd
				print "Tuned parameters kp kd for alt",self.alt_pid.kp,self.alt_pid.kd
				print ".....Waypoint",self.target_count-1,"Reached"
				print " ........Chaning Waypoint!"
				self.pubcmd.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
				#rospy.sleep(1)
			if self.target_count==4:
				self.target_count=0
				count+=1
			self.PublishParms()	
		print "Drone Landing"
		self.pubLand.publish(Empty())
if __name__=='__main__':

	rospy.init_node("AutoPID",anonymous=True)
	pid=Controller()
	rospy.sleep(1.0)
	pid.pubTakeoff.publish(Empty())
	rospy.sleep(5.0)
	#while not rospy.is_shutdown():
	rect_target=[[0,1.5,1.0],[2.0,1.5,1.0],[2.0,0.0,1.0],[0.0,0.0,1.0]] # xy
	#rect_target=[[0,0,1.5],[0,1.5,1.5],[0,1.5,0.8],[0.0,0.0,0.8]] #yz
	pid.DroneModel(rect_target)
