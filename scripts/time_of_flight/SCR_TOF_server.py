#!/usr/bin/env python

from scr_control.srv import *
import rospy
import socket
import time
import sys
import os

class TimeOfFlightServer():

	'''
	INIT FUNCTIONS
	'''

	def __init__(self):
		print("Starting TOF server")
		self.data_location = self.read_config()
		self.server_init()
	
	'''
	COMMAND HANDLERS
	'''

	def read_config(self):
		config = open(os.path.join(os.path.dirname(__file__), 'SCR_TOF_conf.txt'), 'r')
		return config.readline()

	def handle_get_distances(self, req):
		distances = [0]*160*75


		for i in range(18):
			startY = 25
			startX = 0
			
			if (i < 8):
				startY = 50
				startX = i*20
			elif (i == 8):
				startX = 140
				startY = 25
			elif (i < 17):
				startY = 0
				startX = 140 - (i-9)*20

			data_file = open(self.data_location + str(i) + ".txt", 'r')
			y = startY
			for line in data_file:
				lineData = line.split()
				x = startX
				for num in lineData:
					distances[y * 160 + x] = (int(num, 16))
					x += 1
				y += 1

		return TOFGetDistancesAllResponse(distances)

		
	'''
	HELPER FUNCTIONS
	'''

	def server_init(self):
		rospy.init_node("time_of_flight_server")

		lift_service = rospy.Service(
			"get_distances",
			TOFGetDistancesAll,
			self.handle_get_distances)

		rospy.spin()

if (__name__ == "__main__"):
	TimeOfFlightServer()