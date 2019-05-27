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
from playsound import playsound

class pioneer(object):

	def __init__(self):

		rospy.init_node('ListenerTalker',anonymous=False)
		rospy.Subscriber('/RosAria/pose',Odometry,self.odom_callback,queue_size=1)
		rospy.Subscriber("/scan", LaserScan, self.laser_callback,queue_size=1,buff_size=2**25)
		# setup publisher to later on move the pioneer base
		self.pub_move = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
		self.positionx=0.0
		self.positiony=0.0
		self.xdetect=0
		self.ydetect=0
		self.xleftdetect=10
		self.yleftdetect=10
		self.yaw=0.0
		self.vmax=0.3
		self.v=0
		self.flag=0
		self.xref=0
		self.yref=0
		self.teta=0
		self.threshold=0.7
		self.thresholdleft=2
		self.distance=5
		self.distanceleft=0


	def odom_callback(self, data):

		quaternion =(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
		euler = euler_from_quaternion(quaternion)
		#roll = euler[0]
		#pitch = euler[1]
		self.yaw =euler[2]
		self.positionx=data.pose.pose.position.x
		self.positiony=data.pose.pose.position.y
		#print('yaw = ', self.yaw)


	def laser_callback(self, data):
		self.ranges=data.ranges
		self.distance=math.fsum(self.ranges[336:346])/11 
		if math.isnan(self.distance)==True or self.distance<0.6:
			self.distance=5
		self.distanceleft=math.fsum(self.ranges[588:598])/11
		#print('left = ', self.distanceleft)
		

	def detectDoor(self):
		if  self.flag>1 and self.flag<6:
			#print('DIFFERENCE = ', (math.fsum(self.ranges[90:100])/11-math.fsum(self.ranges[70:80])/11))
			if (math.fsum(self.ranges[90:100])/11-math.fsum(self.ranges[70:80])/11>self.portaref) and ( abs(self.positionx-self.xdetect)>1 or abs(self.positiony-self.ydetect)>1):
				self.xdetect=self.positionx
				self.ydetect=self.positiony
				print('DIREITAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
				print('DIFFERENCE = ', (math.fsum(self.ranges[90:100])/11-math.fsum(self.ranges[70:80])/11))
				self.v=0.1
				doorDistance=math.fsum(self.ranges[80:90])/11
				print('DOOR DISTANCE:', doorDistance)
				if math.isnan(doorDistance)==True or doorDistance>1:
					print('OPEN')
					playsound('/home/luislopes/ros_ws/src/Robotics/ros/src/Robotics_ros/aberta_direita.mp3')
				elif(doorDistance<0.9):
					print('CLOSED')
					playsound('/home/luislopes/ros_ws/src/Robotics/ros/src/Robotics_ros/fechada_direita.mp3')
				else:
					print('SEMI')
					playsound('/home/luislopes/ros_ws/src/Robotics/ros/src/Robotics_ros/semi_direita.mp3')
				self.v=self.vmax

			if (self.flag==3 and (self.positiony>10 and self.positiony<16)) or (self.flag==5 and (self.positiony<15 and self.positiony>10)):
				
				if (math.fsum(self.ranges[585:595])/11-math.fsum(self.ranges[605:615])/11>0.04) and (abs(self.positionx-self.xleftdetect)>0.7 or abs(self.positiony-self.yleftdetect)>0.7):
					self.xleftdetect=self.positionx
					self.yleftdetect=self.positiony
					print('DIFFERENCE = ', (math.fsum(self.ranges[585:595])/11-math.fsum(self.ranges[605:615])/11))
					print('ESQUERDAAAAAAAAAAAAAAAAAAAAAAA')
					self.v=0.1
					doorDistance=math.fsum(self.ranges[595:605])/11
					print('DOOR DISTANCE:', doorDistance)
					if math.isnan(doorDistance)==True or doorDistance>1.3:
						print('OPEN')
						playsound('/home/luislopes/ros_ws/src/Robotics/ros/src/Robotics_ros/aberta_esquerda.mp3')
					elif(doorDistance<1.2):
						print('CLOSED')
						playsound('/home/luislopes/ros_ws/src/Robotics/ros/src/Robotics_ros/fechada_esquerda.mp3')
					else:
						print('SEMI')
						playsound('/home/luislopes/ros_ws/src/Robotics/ros/src/Robotics_ros/semi_esquerda.mp3')
					self.v=self.vmax

	def correct(self):
		#print('WALL AT: ', self.distance)
		if self.distance>self.threshold:
			wallDistance=math.fsum(self.ranges[82:88])/6
			#print('wallDistance', wallDistance)
			if math.isnan(wallDistance)==True:
				wallDistance=self.ref

			if wallDistance-self.ref<-0.025:
				self.v=0.3			
				self.w=0.05
				self.move_forward()
			elif wallDistance-self.ref>0.025:
				self.v=0.3
				self.w=-0.05
				self.move_forward()
			elif abs(wallDistance-self.ref)<0.025:
				self.v=self.vmax
				self.w=0
				self.move_forward()

		if self.distance<=self.threshold and self.flag<5: #or (math.isnan(self.distanceleft) and (self.positionx>self.xleftdetect and self.positiony>self.yleftdetect) and self.flag==3):
			self.stop()
			if self.flag==4 and self.yaw>self.teta:
				#print('YAWWWWWW')
				while abs(self.yaw)>abs(self.teta):
					self.w=0.3
					self.move_left()
				self.v=self.vmax
				self.w=0
				self.flag=self.flag+1
				#print('FLAG: ', self.flag)
			else:
				print('oooooooooooooooooooooooooooooooooooooooooooooooooooooo')
				while self.yaw<self.teta:			
					self.w=0.3
					self.move_left()
				self.v=self.vmax
				self.w=0
				self.flag=self.flag+1
				print('FLAG: ', self.flag)
		if self.flag==5 and self.positiony<4:
			print('ENTROUUU')
			self.stop()
			self.flag=self.flag+1

	def stop(self):
		twist_msg = Twist()
      
		twist_msg.linear.x = 0.0
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0
       
		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = 0.0
		self.pub_move.publish(twist_msg)

	def move_forward(self):
		self.detectDoor()
		twist_msg = Twist()
		#print('w=', self.w)
		twist_msg.linear.x = self.v
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0
       
		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = self.w

        # publish Twist message to move the robot
		self.pub_move.publish(twist_msg)
	
	def move_left(self):

		twist_msg = Twist()
      
		twist_msg.linear.x = 0.0
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0
       
		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = self.w
        # publish Twist message to move the robot
		self.pub_move.publish(twist_msg)

	def move_right(self):

		twist_msg = Twist()
      
		twist_msg.linear.x = 0.0
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0
       
		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = -self.w

        # publish Twist message to move the robot
		self.pub_move.publish(twist_msg)

	def trajectx_1(self):
		
		if self.positionx<self.xref:
			self.v=self.vmax
			self.w=0
			self.move_forward()
		else:
			while self.yaw<self.teta:
				self.w=0.3
				self.move_left()
			self.flag=self.flag+1

	def trajectx_2(self):		
		if self.positionx>self.xref:
			self.v=self.vmax
			self.w=0
			self.move_forward()
		else:
			while self.yaw<self.teta:
				self.w=0.3
				self.move_right()
			self.flag=self.flag+1
			
	def trajecty_1(self):
		
		if self.positiony<self.yref:
			self.v=self.vmax
			self.w=0
			self.move_forward()
		else:
			while self.yaw>self.teta:
				self.w=0.3
				self.move_right()
			self.flag=self.flag+1

	def trajecty_2(self):
		
		if self.positiony>self.yref:
			self.v=self.vmax
			self.w=0
			self.move_forward()
		else:
			while self.yaw<self.teta:
				self.w=0.3
				self.move_right()
			self.flag=self.flag+1
			
	def move(self):
		while not rospy.is_shutdown():
	        # base needs this msg to be published constantly for the robot to keep moving so we publish in a loop
			
	        # while the distance from the robot to the walls is bigger than the defined threshold keep moving forward
			if self.flag==0:
				self.xref=4.2
				self.yref=0
				self.teta=1.43
				self.trajectx_1()				
				
							
			if self.flag==1:
				self.xref=4.4
				self.yref=3.7
				self.teta=0
				self.trajecty_1()

			if self.flag==2:
				self.ref=0.83
				self.rigth=-1.3
				self.center=0
				self.teta=1.42
				self.portaref=0.04
				self.correct()

			if self.flag==3:
				self.ref=0.63
				self.rigth=0.2
				self.center=1.42
				self.left=3.0
				self.teta=3
				self.portaref=0.04
				self.correct()

			if self.flag==4:
				self.ref=0.63
				self.rigth=1.47
				self.center=3.1
				self.left=-1.47
				self.teta=-1.7
				self.portaref=0.04
				self.correct()

			if self.flag==5:
				self.ref=0.7
				self.rigth=3.1
				self.center=-1.42
				self.left=0
				self.portaref=0.04
				self.correct()

			if self.flag==6:
				self.xref=4.2
				self.yref=0.5
				self.teta=3.1
				self.trajecty_2()

			if self.flag==7:
				self.xref=0
				self.yref=0
				self.trajectx_2()
			
	        # sleep for a small amount of time
			rospy.sleep(0.1)

def main():

	# create object of the class pioneer
	obj = pioneer()
    # call move method of class pioneer
	obj.move()