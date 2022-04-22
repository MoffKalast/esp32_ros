#!/usr/bin/env python3

import rospy
import time
import websocket
import threading
import sys
import math

from geometry_msgs.msg import Twist

MIN_SPD = 130
MAX_SPD = 255
DEADMAN = 0.3 # seconds

def clamp(val, minval, maxval):
	if val < minval:
		return minval
	if val > maxval:
		return maxval
	return val

def diffdrive(x,  y):

		x = clamp(x, -1.0, 1.0)
		y = clamp(y, -1.0, 1.0)

		# First Compute the angle in deg
		# First hypotenuse
		z = math.sqrt(x * x + y * y)

		# angle in radians
		rad = math.acos(math.fabs(x) / z)

		# and in degrees
		angle = rad * 180 / math.pi

		# Now angle indicates the measure of turn
		# Along a straight line, with an angle o, the turn co-efficient is same
		# this applies for angles between 0-90, with angle 0 the coeff is -1
		# with angle 45, the co-efficient is 0 and with angle 90, it is 1
		tcoeff = -1 + (angle / 90) * 2
		turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
		turn = round(turn * 100, 0) / 100

		# And max of y or x is the movement
		mov = max(math.fabs(y), math.fabs(x))

		# First and third quadrant
		if (x >= 0 and y >= 0) or (x < 0 and y < 0):
			rawLeft = mov
			rawRight = turn
		else:
			rawRight = mov
			rawLeft = turn

		rawRight = round(rawRight * 255)
		rawLeft = round(rawLeft * 255)

		if y < 0:
			return [-rawLeft, -rawRight]

		return [rawRight, rawLeft]

class ESP32:
	def __init__(self):
		rospy.init_node('esp32_bot', anonymous=False)

		self.left_mot = 0
		self.right_mot = 0

		self.update_vel = False

		self.ws = websocket.WebSocket()
		self.ws.connect("ws://192.168.4.1:8080")

		print("Connected.")

		self.cmdsub = rospy.Subscriber("/cmd_vel", Twist, self.velocity)
		self.m_time = time.time()

	def set_motors(self, left, right):
		left_dir = 0
		left_spd = 0

		right_dir = 0
		right_spd = 0

		if left > 0:
			left_dir = 2
		elif left < 0:
			left_dir = 1

		if right > 0:
			right_dir = 2
		elif right < 0:
			right_dir = 1

		try:
			self.ws.send_binary(bytearray([left_dir, abs(left), right_dir, abs(right)]))
		except:
			print(sys.exc_info()[0])

			connected = False
			while not connected:
				try:
					self.ws = websocket.WebSocket()
					self.ws.connect("ws://192.168.4.1:8080")
					connected = True
				except:
					print(sys.exc_info()[0])

				time.sleep(0.5)
		
	def update(self):
		if rospy.get_time() - self.m_time > DEADMAN:
			self.left_mot = 0
			self.right_mot = 0
			self.set_motors(0,0)

		elif self.update_vel:
			self.update_vel = False
			self.set_motors(self.left_mot, self.right_mot)	

	def velocity(self, msg):
		self.m_time = time.time()

		if msg.angular.z == 0 and msg.linear.x == 0:
			self.left_mot = 0
			self.right_mot = 0
			return

		left_vel, right_vel = diffdrive(-msg.angular.z, msg.linear.x)

		left_spd = abs(left_vel)
		right_spd = abs(right_vel)

		if left_spd != 0:
			if left_spd < MIN_SPD:
				left_vel = math.copysign(MIN_SPD, left_vel)
			elif left_spd > MAX_SPD:
				left_vel = math.copysign(MAX_SPD, left_vel)

		if right_spd != 0:
			if right_spd < MIN_SPD:
				right_vel = math.copysign(MIN_SPD, right_vel)
			elif right_spd > MAX_SPD:
				right_vel = math.copysign(MAX_SPD, right_vel)

		self.left_mot = left_vel
		self.right_mot = right_vel
		self.update_vel = True

	def cleanup(self):
		self.run = False
		self.ws.close()

try:
	esp = ESP32()
	rospy.on_shutdown(esp.cleanup)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		esp.update()
		rate.sleep()
except Exception as e:
	print(e)