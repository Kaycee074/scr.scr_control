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
		data = self.get_sensorData(req.sensor_id)

		return TOFGetDistancesResponse(data)

	#160 x 75
	def handle_getDistancesAll(self, req):
		distances =  [0]*(160*75)
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
		
			data = self.get_sensorData(i)

			for j in range(len(data)):
				x = (j % 20) + startX
				y = (j / 20) + startY
				distances[x+y*160] = data[j]

		return TOFGetDistancesAllResponse(distances)


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
		return TOFSetCountingResponse()


	'''
	HELPER FUNCTIONS
	'''

	def get_sensorData(self, sensor_id):
		s = self.establish_connection()
		s.send("get_data"+str(sensor_id))
		data = s.recv(3000)
		s.shutdown(socket.SHUT_RDWR)
		s.close()


		data = data.split(",")
		data = [int(float(x)) for x in data]

		return data

	def server_init(self):
		rospy.init_node("time_of_flight_server")

		get_distances_service = rospy.Service(
			"get_distances",
			TOFGetDistances,
			self.handle_getDistances)

		get_distances_all_service = rospy.Service(
			"get_distances_all",
			TOFGetDistancesAll,
			self.handle_getDistancesAll)

		set_counting_service = rospy.Service(
			"set_counting",
			TOFSetCounting,
			self.handle_setCounting)

		rospy.spin()

if (__name__ == "__main__"):
	TimeOfFlightServer()