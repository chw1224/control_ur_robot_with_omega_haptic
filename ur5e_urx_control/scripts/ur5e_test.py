#!/usr/bin/env python3

import rospy, sys
from geometry_msgs.msg import Vector3
import math3d as m3d
import urx
import time
import numpy as np
import math





class ur5e_robot:

	def __init__(self, ur5e_port, tcp=((0,0,0.1,0,0,0)), payload_m=0.0, payload_location=(0,0,0)):
		try:
			print("Connecting to UR5e")
			self.ur5e = urx.Robot(ur5e_port)
			print(self.ur5e)
			self.connect_success = True
		except:
			print("Cannot connect UR5e")
			self.connect_success = False
			return
			
		if self.ur5e.host == ur5e_port:
			self.ur5e.set_tcp(tcp)
			self.ur5e.set_payload(payload_m, payload_location)
			trx = m3d.Transform()
			self.ur5e.set_csys(trx)
			
		
		self.get_initial_pose()
		self.robot_goal_position = [0.0, 0.0, 0.0]
		
	def get_initial_pose(self):
		pose = self.ur5e.get_pose()
		self.initial_pose = [round(pose.pos[0],2),round(pose.pos[1],2),round(pose.pos[2],2)]
		
	
	def hapticDeltaPositionCallback(self, data):
		#self.robot_goal_position[0] = -data.y
		#self.robot_goal_position[2] = data.z
		pass
		
	def hapticVelocityCallback(self, data):
		self.robot_goal_position[0] = -data.y*0.05
		self.robot_goal_position[2] = data.z*0.05
		
	def control_ur5e_speed(self):
		speeds = [0.0,0.0,0.0,0.0,0.0,0.0]
		speeds[0] = self.robot_goal_position[0]
		speeds[2] = self.robot_goal_position[2]
		self.ur5e.speedl(speeds, 0.3, 2)
		
		
		# self.goal_pose = self.ur5e.getl()
		
		# self.goal_pose[0] = self.initial_pose[0] + self.robot_goal_position[0]
		# self.goal_pose[1] = self.initial_pose[1] + self.robot_goal_position[1]
		# self.goal_pose[2] = self.initial_pose[2] + self.robot_goal_position[2]
		# self.ur5e.movel(self.goal_pose, acc=0.3, vel=0.05, wait=False) 
		
		
		
		
	def init(self):
		rospy.init_node('ur5e_robot', anonymous=False)
		rospy.Subscriber("/haptic/delta_position", Vector3, self.hapticDeltaPositionCallback)
		rospy.Subscriber("/haptic/velocity", Vector3, self.hapticVelocityCallback)
		
	def run(self):
		rate = rospy.Rate(5)
		while not rospy.is_shutdown():
			try:
				self.control_ur5e_speed()
			except KeyboardInterrupt:
				print("^C")
				break
			rate.sleep()
			
	def get_success(self):
		return self.connect_success

def main():
	ur5e_port = "192.168.1.101"
	ur5e = ur5e_robot(ur5e_port)
	success_ = ur5e.get_success()
	if success_:
		ur5e.init()
		ur5e.run()
	else :
		print("Error!")
	
		
if __name__=='__main__':
	main()
