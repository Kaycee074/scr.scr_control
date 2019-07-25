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
		self.data_location = '/home/arunas/tof_control/SCR/output-'
		self.server_init()

	'''
	COMMAND HANDLERS
	'''

	def read_config(self):
		config = open(os.path.join(os.path.dirname(__file__), 'SCR_TOF_conf.txt'), 'r')
		return config.readline()

	def handle_get_distances_all(self, req):
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
				x = startX + 19
				for num in lineData:
					try:
						distances[ y * 160 + x ] = int(num)
					#	distances[y * 160 + x] = (int(num, 32))
					#	distances[y][x] = (int(num,16))
					except:
						print("x =", x, "\ny = ", y)
					x -= 1
				y += 1
		return TOFGetDistancesAllResponse(distances)


	def handle_get_distances(self, req):
		distances = [0]*20*25
		data_file = open(self.data_location + str(req.sensor_id) + ".txt", 'r')
		y = 0
		for line in data_file:
			lineData = line.split()
			x = 19
			for num in lineData:
				try:
					distances[ y * 20 + x ] = int(num)
				except:
					print("x =", x, "\ny = ", y)
				x -= 1
			y += 1
		return TOFGetDistancesResponse(distances)

	'''
	HELPER FUNCTIONS
	'''

	def server_init(self):
		rospy.init_node("time_of_flight_server")

		distances_service = rospy.Service(
			"get_distances",
			TOFGetDistances,
			self.handle_get_distances)

		distances_all_service = rospy.Service(
			"get_distances_all",
			TOFGetDistancesAll,
			self.handle_get_distances_all)

		rospy.spin()

if (__name__ == "__main__"):
	TimeOfFlightServer()
