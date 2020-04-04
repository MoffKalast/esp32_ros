#!/usr/bin/env python
from __future__ import print_function

import rospy
import time
import websocket
import threading
import sys

from geometry_msgs.msg import Twist

class ESP32:
	def __init__(self):
		rospy.init_node('esp32_car', anonymous=False)

		self.spd = 0
		self.turn = 0
		self.update_vel = False
		self.run = True

		self.ws = websocket.WebSocket()
		self.ws.connect("ws://192.168.1.8")

		print("Connected.")

		self.cmdsub = rospy.Subscriber("/cmd_vel", Twist, self.velocity)

		self.thread = threading.Thread(target=self.update)
		self.thread.start()

	def set_motors(self):

		motor_dir = 0
		motor_spd = 0

		turn_dir = 0
		turn_angle = 0

		if self.spd > 0:
			motor_dir = 1
		elif self.spd < 0:
			motor_dir = 2

		if self.turn > 0:
			turn_dir = 1
		elif self.turn < 0:
			turn_dir = 2

		motor_spd = int(abs(self.spd)*255)
		turn_angle = int(abs(self.turn)*255)

		if motor_spd > 255:
			motor_spd = 255
		elif motor_spd < 0:
			motor_spd = 0

		if turn_angle > 255:
			turn_angle = 255
		elif turn_angle < 0:
			turn_angle = 0

		try:
			self.ws.send_binary(bytearray([motor_dir, motor_spd, turn_dir, turn_angle]))
		except:
			print(sys.exc_info()[0])

			connected = False

			while not connected:
				try:
					self.ws = websocket.WebSocket()
					self.ws.connect("ws://192.168.1.8")
					connected = True
				except:
					print(sys.exc_info()[0])

				time.sleep(0.5)
		

	def update(self):
		while self.run:

			if self.update_vel:
				self.update_vel = False
				self.set_motors()

			time.sleep(0.1)


	def velocity(self, msg):
		self.spd = msg.linear.x
		self.turn = msg.angular.z
		self.update_vel = True


	def cleanup(self):
		self.run = False
		self.ws.close()

try:
	esp = ESP32()
	rospy.on_shutdown(esp.cleanup)
	rospy.spin()
except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)