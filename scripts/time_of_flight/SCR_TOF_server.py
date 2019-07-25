#!/usr/bin/env python

from scr_control.srv import *
import rospy
import socket
import time
import sys
import os
import numpy as np

class TimeOfFlightServer():

	'''
	INIT FUNCTIONS
	'''

	def __init__(self):
		print("Starting TOF server")
		self.data_location = '/home/arunas/tof_control/SCR/output-'
		self.server_init()
        self.data = []
        for i in range(18):
            self.data.append(np.zeros((20, 25)))
	'''
	COMMAND HANDLERS
	'''

	def read_config(self):
		config = open(os.path.join(os.path.dirname(__file__), 'SCR_TOF_conf.txt'), 'r')
		return config.readline()

    def read_sensor(self, i):
        distances = np.zeros((20, 25))
        try:
    		data_file = open(self.data_location + str(i) + ".txt", 'r')
    		y = 0
    		for line in data_file:
    			lineData = line.split()
    			x = 19
    			for num in lineData:
   					distances[ y * 20 + x ] = int(num)
    				x -= 1
    			y += 1
            self.data[i] = distances
    	    return distances 							
        except:
            return self.data[i]

	def handle_get_distances_all(self, req):
		distances = np.zeros((160, 75))

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
            sensor_data = self.read_sensor(i)
            distances[startX:startX+20, startY:startY+25] = sensor_data

		return TOFGetDistancesAllResponse(distances)


	def handle_get_distances(self, req):
		distances = self.read_sensor(req.sensor_id)
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
