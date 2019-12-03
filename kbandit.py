#!/usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage

import numpy as np
import time
import miro2 as miro
import os
import math

MAX_TIME = 10.0

#Generate a fake enum for joint arrays
tilt, lift, yaw, pitch = range(4)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
freq, volume, duration = range(3)
front_left, mid_left, rear_left, front_right, mid_right, rear_right = range(6)

class MiroController:

	def __init__( self ):
		print "Initializing the controller"
		self.actions = [ self.earWiggle, self.tailWag, self.rotate, self.nod ]

		# Set robot name
		topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")
		# Python needs to initialise a ROS node for publishing data
		rospy.init_node("kbandit", anonymous=True)
		# Define ROS publishers
		self.pub_cmd_vel = rospy.Publisher(topic_root + "/control/cmd_vel", TwistStamped, queue_size=0)
		self.pub_cos = rospy.Publisher(topic_root + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)
		self.pub_illum = rospy.Publisher(topic_root + "/control/illum", UInt32MultiArray, queue_size=0)
		self.pub_kin = rospy.Publisher(topic_root + "/control/kinematic_joints", JointState, queue_size=0)

		# Initializing object for data publishing
		self.velocity = TwistStamped()
		self.kin_joints = JointState()
		self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
		self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
		self.cos_joints = Float32MultiArray()
		self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.illum = UInt32MultiArray()
		self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]


	# Actions
	def earWiggle( self, t ):
		A = 1.5
		w = 2*np.pi*0.2
		f = lambda t: A*np.cos(w*t)

		self.cos_joints.data[left_ear] = f(t)
		self.cos_joints.data[right_ear] = f(t)
		self.pub_cos.publish(self.cos_joints)

		print "wiggle"

		if t > MAX_TIME:
			return False
		else:
			return True

	def tailWag( self, t ):
		A = 1.5
		w = 2*np.pi*0.2
		f = lambda t: A*np.cos(w*t)
		self.cos_joints.data[wag] = f(t)
		self.cos_joints.data[droop] = 0.0

		self.pub_cos.publish(self.cos_joints)

		print "wag"

		if t > MAX_TIME:
			return False
		else:
			return True

	def rotate( self, t ):

		if t > MAX_TIME:
			l_val = 0.0
			r_val = 0.0
			r = False
		else:
			l_val = 1.0
			r_val = 0.0	
			r = True

		wheel_speed = [l_val, r_val]
		(dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)
		self.velocity.twist.linear.x = dr
		self.velocity.twist.angular.z = dtheta

		self.pub_cmd_vel.publish(self.velocity)

		print "rotate"

		return r


	def nod( self, t ):
		print "nod"

		return False

	# Listeners
	def touchListener( self ):
		pass

	# Main loop
	def run( self ):
		running = True
		r = np.random.randint( 0, len(self.actions) )
		t = 0.0
		h = 0.1

		print "Starting the loop"

		while( running ):
			if not self.actions[r]( t ):
				print "Action Finished, changing action"
				# Action selection
				r = np.random.randint( 0, len(self.actions) )
				t = 0.0
			
			t = t + h			
			time.sleep(0.01)

if __name__ == "__main__":
	mc = MiroController()
	mc.run()