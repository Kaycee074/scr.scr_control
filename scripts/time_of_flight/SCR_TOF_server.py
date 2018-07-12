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
		distances = []
		room_length = 0
		data_file = open(self.data_location, 'r')

		for line in data_file:
			lineData = line.split()
			if room_length == 0:
				room_length = len(lineData)
			for num in lineData:
				distances.append(int(num))

		return TOFGetDistancesResponse(distances)
		
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