#!/usr/bin/env python3

import math
import rospy
import time
import cv2
import numpy as np
import tf2_ros

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3Stamped

from dnn_detect.msg import DetectedObjectArray

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

def clamp(val, minval, maxval):
	if val < minval:
		return minval
	if val > maxval:
		return maxval
	return val

class Automatic:
	def __init__(self):
		rospy.init_node('automatic', anonymous=False)

		self.buffer = tf2_ros.Buffer(rospy.Time(30))
		self.listener = tf2_ros.TransformListener(self.buffer)
		self.broadcaster = tf2_ros.TransformBroadcaster()

		self.linear = 0
		self.turn = 0

		self.lastdetected = 0
		self.lastturn = 0

		self.bridge = CvBridge()
		self.run = True

		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.dnn_sub = rospy.Subscriber("/dnn_objects", DetectedObjectArray, self.dnn_objects)

	def dnn_objects(self, msg):
		if len(msg.objects) >= 1:

			people = []
			for x in msg.objects:
				if x.class_name == "person":
					people.append(x)

			if len(people) == 0:
				return

			x = (people[0].x_min + people[0].x_max) / 2.0
			x = (x / 640.0) - 0.5

			print(x)
			self.lastturn = math.copysign(1, x)
			self.lastdetected = rospy.get_time()

			#linear = 0.66#clamp(linear, -0.7, 0.7)
			
			self.turn = -clamp(x*8.0, -1.0, 1.0)
			self.twist(0.68,self.turn)


	def update(self):
		delta = rospy.get_time() - self.lastdetected
		if delta > 2.0 and delta < 5.0:
			self.twist(-0.68, self.lastturn)
		elif delta > 5.0 and delta < 6.0:
			self.twist(0.68, -self.lastturn)

	def twist(self, linear, angular):
		cmd = Twist()
		cmd.linear.x = linear
		cmd.angular.z = angular
		self.vel_pub.publish(cmd)

try:
	esp = Automatic()

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		esp.update()
		rate.sleep()

	rospy.spin()
except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
