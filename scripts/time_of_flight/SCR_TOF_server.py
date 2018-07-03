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

	def handle_getDistances(self, req):
		s = self.establish_connection()
		s.send("get_data"+str(req.sensor_id))
		data = s.recv(3000)
		s.shutdown(socket.SHUT_RDWR)
		s.close()

		data = data.split(",")
		data = [int(float(x)) for x in data]

		return TOFGetDistancesResponse(data)


	def handle_setCounting(self, req):
		if req.mode:
			s = self.establish_connection()
			s.send("start_counting")
			#data = s.recv(3000)
			s.shutdown(socket.SHUT_RDWR)
			s.close()

		else:
			s = self.establish_connection()
			s.send("stop_counting")
			#data = s.recv(3000)s
			s.shutdown(socket.SHUT_RDWR)
			s.close()


	'''
	HELPER FUNCTIONS
	'''

	def server_init(self):
		rospy.init_node("time_of_flight_server")

		lift_service = rospy.Service(
			"get_distances",
			TOFGetDistances,
			self.handle_getDistances)

		lift_service = rospy.Service(
			"set_counting",
			TOFSetCounting,
			self.handle_setCounting)

		rospy.spin()

if (__name__ == "__main__"):
	TimeOfFlightServer()