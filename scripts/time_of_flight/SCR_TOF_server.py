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
		self.address = self.read_config()
		self.server_init()
	
	def read_config(self):

		config = open(os.path.join(os.path.dirname(__file__), 'TOF_conf.txt'))
		line = config.readline()
		line = line.rstrip()
		config.close()

		return line

	def establish_connection(self):
		port = 3660
		print("Trying to reach", port, self.address)
		try:
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			s.connect((self.address,port))
		except:
			#rs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			#rs.connect((self.address, 57011))
			print("Connection refused on " + self.address)
		return s

	
	'''
	COMMAND HANDLERS
	'''

	def read_config(self):
		config = open(os.path.join(os.path.dirname(__file__), 'SCR_TOF_conf.txt'), 'r')
		return config.readline()

	def handle_getHeatmap(self, req):
		distances = []
		room_length = 0
		data_file = open(self.data_location, 'r')

		for line in data_file:
			lineData = line.split()
			if room_length == 0:
				room_length = len(lineData)
			for num in lineData:
				distances.append(int(num))

		return GetHeatmapResponse(distances, room_length)

	def handle_setCounting(self, req):
		if req.mode:
			print("Starting to update TOF sensors...")
			s = self.establish_connection()
			s.send("start_counting")
			#data = s.recv(3000)
			s.shutdown(socket.SHUT_RDWR)
			s.close()

		else:
			print("Stopping TOF sensors...")
			s = self.establish_connection()
			s.send("stop_counting")
			#data = s.recv(3000)
			s.shutdown(socket.SHUT_RDWR)
			s.close()


	'''
	HELPER FUNCTIONS
	'''

	def server_init(self):
		rospy.init_node("time_of_flight_server")

		lift_service = rospy.Service(
			"get_distances",
			GetHeatmap,
			self.handle_getHeatmap)

		lift_service = rospy.Service(
			"set_counting",
			TOFSetCounting,
			self.handle_setCounting)

		rospy.spin()

if (__name__ == "__main__"):
	TimeOfFlightServer()