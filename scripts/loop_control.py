#!/usr/bin/env python
#coding=utf-8

"""
This program makes xbot automnomous get the target point in map. The main funcution is testing the base movements of xbot such as moving forward and rotating.
Copyright (c) 2016 Peng Wang (Rocwang).  All rights reserved.
This program is free software; you can redistribute or modify it.
More details about xbot robot platform is available in http://wiki.ros.org/Robots/Xbot.
"""

import rospy, sys, termios, tty, math, time, cv2, numpy, PyKDL
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped,Point
from geometry_msgs.msg import Quaternion

class LoopControl(object):
	"""docstring for LoopControl"""
	def __init__(self):
		self.help="""

		"""
		# if rospy.has_param("~cascPath"):
		# 	self.cascPath = rospy.get_param("~cascPath")
		# else:
		# 	rospy.set_param("~cascPath","../scripts/haarcascade_frontalface_default.xml")
		self.PathBias=0.01
		self.AngularFree=0.1745
		self.AngularBias=0.3
		self.MaxAngularSP=0.4
		self.control_pub=rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
		rospy.Subscriber("clicked_point",PointStamped, self.target_callback)
		rospy.Subscriber("odom",Odometry,self.odom_callback)
		self.target=PointStamped()
		rospy.spin()


	def target_callback(self,point):
		self.target=point





	def odom_callback(self,odom):
		if self.target==None:
			return
		cmd=Twist()
		cmd=self.DiffControl(odom)
		self.control_pub.publish(cmd)

	def AngularDrift(self, odom):
		x_drift = self.target.point.x - odom.pose.pose.position.x
		y_drift = self.target.point.y - odom.pose.pose.position.y
		angular_drift = numpy.arcsin(y_drift / numpy.sqrt(x_drift**2 + y_drift**2))

		if x_drift > 0 and y_drift < 0:
			angular_drift = angular_drift

		if x_drift > 0 and y_drift > 0:
			angular_drift = angular_drift

		if x_drift < 0 and y_drift < 0:
			angular_drift = -angular_drift - numpy.pi

		if x_drift < 0 and y_drift > 0:
			angular_drift = numpy.pi - angular_drift

		return (angular_drift, x_drift, y_drift)


	def GoalOrientation(self, theta):
		orientation = Quaternion()

		if -numpy.pi < theta < -numpy.pi*2.0/3.0:
			orientation.z = -numpy.sin(theta/2.0)
			orientation.w = -numpy.cos(theta/2.0)

		else:
			orientation.z = numpy.sin(theta/2.0)
			orientation.w = numpy.cos(theta/2.0)

		return orientation


	def GetAngle(self, quat):
		rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
		return rot.GetRPY()[2]


	def DiffControl(self, odom):
		cmd = Twist()
		CrossFire = False

		(angular_drift, x_drift, y_drift) = self.AngularDrift(odom)

		Gorientation = self.GoalOrientation(angular_drift)
		linear = 0.2 #numpy.sqrt(x_drift**2 + y_drift**2)

		GoalAngle = self.GetAngle(Gorientation)
		OdomAngle = self.GetAngle(odom.pose.pose.orientation)

		#如果goal和当前朝向相同
		if GoalAngle * OdomAngle >= 0:
			cmdtwist = GoalAngle - OdomAngle

		#如果不同边（存在符号更变）：只需要以最快速度过界，从而达到同边即可
		else:
			CrossFire = True
			GoalToCPP = numpy.pi - abs(GoalAngle)
			OdomToCPP = numpy.pi - abs(OdomAngle)

			GoalToCPO = abs(GoalAngle)
			OdomToCPO = abs(OdomAngle)

		# 不同边,判断临界线
		# goal 临界线
			if GoalToCPP < GoalToCPO:
				GCriticalLine = numpy.pi

			else:
				GCriticalLine = 0
			# Odom 临界线
			if OdomToCPP < GoalToCPO:
				OCriticalLine = numpy.pi

			else:
				OCriticalLine = 0

			# 不同边,同临界线
			if OCriticalLine == GCriticalLine:
				rospy.loginfo('reg as same critical line')
				# 不同边, pi 临界线 临界角
				if OCriticalLine == numpy.pi:
					rospy.loginfo('changing line in pi')
					if GoalAngle >= 0:
						rospy.loginfo('goal upon line in pi')
						OdomToC = -abs(abs(GoalAngle) - abs(OdomAngle))
					else:
						rospy.loginfo('goal under line in pi')
						OdomToC = abs(abs(GoalAngle) - abs(OdomAngle))

				# 不同边,0 临界线 临界角
				elif OCriticalLine == 0: #
					rospy.loginfo('changing line in 0')
					if GoalAngle >= 0:
						rospy.loginfo('goal upon line in pi')
						OdomToC = abs(abs(GoalAngle) - abs(OdomAngle))
					else:
						rospy.loginfo('goal under line in pi')
						OdomToC = -abs(abs(GoalAngle) - abs(OdomAngle))
				else:
					rospy.loginfo('differ changing point in ???')

			# 不同边, 不同临界线
			else:
				rospy.loginfo('reg as differ critical line')
				# 不同边,  pi 临界线
				if OdomToCPP + GoalToCPP < OdomToCPO + GoalToCPO:
					rospy.loginfo('differ changing point in pi')
					OdomToC = OdomToCPP
				# 不同边, 0 临界线
				elif OdomToCPP + GoalToCPP >= OdomToCPO + GoalToCPO:
					rospy.loginfo('differ changing point in 0')
					OdomToC = OdomToCPO
				else:
					rospy.loginfo('differ changing point in ???')

			# 不同边, 过临界线，转角速度
			if abs(OdomToC) <= 2*abs(self.AngularBias):
				cmdtwist = OdomToC + 0.1 * OdomToC/ (abs(OdomToC))

			else:
				cmdtwist = abs(self.MaxAngularSP) * OdomToC/abs(OdomToC)

		# 是当前坐标否在误差允许之内
		if abs(x_drift) > self.PathBias or abs(y_drift) > self.PathBias:
			if abs(cmdtwist) >= self.AngularBias:
				rospy.loginfo('in position twist')
				cmd.angular.z = cmdtwist #self.MaxAngularSP

			elif self.AngularFree < abs(cmdtwist) < self.AngularBias:
				rospy.loginfo('small circle')
				cmd.angular.z = cmdtwist
				cmd.linear.x = linear

			elif self.AngularFree >= abs(cmdtwist):
				if CrossFire:
					rospy.loginfo('in position twist')
					cmd.angular.z = cmdtwist
				else:
					rospy.loginfo('forward')
					cmd.linear.x = linear

			else:
				pass

		else:
			rospy.loginfo('robot in goal position')
			if self.AngularFree < abs(cmdtwist):
				#cmd.angular.z = cmdtwist
				pass

			else:
				rospy.loginfo('robot in goal orientation')








if __name__ == '__main__':
	rospy.init_node('loop_control')
	try:
		rospy.loginfo('initialization system for loop control...')
		LoopControl()
		print 'process loop control done and quit.'
	except rospy.ROSInterruptException:
		rospy.loginfo('node loop_control termindated.')





