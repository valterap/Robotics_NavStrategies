#!/usr/bin/env python


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import random
import numpy as np


class pioneer(object):

	def __init__(self):

		rospy.init_node('ListenerTalker',anonymous=False)
		rospy.Subscriber('/RosAria/pose',Odometry,self.odom_callback,queue_size=1)
		#rospy.Subscriber("/scan", LaserScan, self.laser_callback,queue_size=1)
		# setup publisher to later on move the pioneer base
		self.pub_move = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
		self.positionx=0.0
		self.positiony=0.0
		self.yaw=0.0
		self.vmax=0.3
		self.v=self.vmax
		self.flag1=0
		self.flag2=0
		self.flag3=0

	def odom_callback(self, data):

		quaternion =(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
		euler = euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		self.yaw = euler[2]
		self.positionx=data.pose.pose.position.x
		self.positiony=data.pose.pose.position.y

	#def laser_callback(self, msg):


	def set_controls(self):
		print('setscontrols')
		dx=math.fabs(self.posx_ref-self.positionx)
		dy=math.fabs(self.posy_ref-self.positiony)
		#dx=self.v*math.cos(self.yaw)
		#dy=self.v*math.sin(self.yaw)
		print('dx: ',dx)
		print('dy: ',dy)
		self.yaw_ref=math.atan2(dy,dx)
		print("yaw_ref: ",self.yaw_ref)
		d_teta=self.yaw_ref-self.yaw
		self.w=d_teta
		self.correct()

	def correct(self):
		
		print("w:",self.w)
		print("yaw: ", self.yaw)

		if self.w>0:
			while self.yaw_ref-self.yaw>0:
				print(self.yaw)
				self.move_left()
		else:
			while self.yaw_ref-self.yaw<0:
				self.move_right()


	def stop(self):

		twist_msg = Twist()
        
		twist_msg.linear.x = 0.0
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0

		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = 0.0

        # publish Twist message to stop the robot
		self.pub_move.publish(twist_msg)

	def move_forward(self):

		twist_msg = Twist()
      
		twist_msg.linear.x = self.v
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0
       
		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = 0.0

        # publish Twist message to move the robot
		self.pub_move.publish(twist_msg)
	
	def move_left(self):

		twist_msg = Twist()
      
		twist_msg.linear.x = 0.0
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0
       
		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = 0.3
        # publish Twist message to move the robot
		self.pub_move.publish(twist_msg)

	def move_right(self):

		twist_msg = Twist()
      
		twist_msg.linear.x = 0.0
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0
       
		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = -0.3

        # publish Twist message to move the robot
		self.pub_move.publish(twist_msg)

	def move(self):
		while not rospy.is_shutdown():
	        # base needs this msg to be published constantly for the robot to keep moving so we publish in a loop

	        # while the distance from the robot to the walls is bigger than the defined threshold keep moving forward
			if self.flag1==0:
				if self.positionx<4.1:
					self.move_forward()
				else:
					while self.yaw<1.45:
						self.move_left()
					self.flag1=1			

			if self.flag2==0:
				if self.positiony<3.8:
					self.move_forward()
				else:
					while self.yaw>0.2:
						self.move_right()

					self.posx_ref=4.1
					self.posy_ref=3.8
					self.set_controls()
					self.flag2=1

			if self.flag3==0:
				if self.positionx<18:
					
					self.move_forward()
				else:
					self.flag3=1

	        # sleep for a small amount of time
			rospy.sleep(0.1)

def main():

	# create object of the class pioneer
	obj = pioneer()
    # call move method of class pioneer
	obj.move()


