#!/usr/bin/env python

from scr_control.srv import *
import rospy
import socket
import time
import sys
import os

class ColorSensorServer():

	'''
	INIT FUNCTIONS
	'''

	def __init__(self):
		self.address = self.initialize_sensors()
		self.COS_server_init()

	def initialize_sensors(self):

		config = open(os.path.join(os.path.dirname(__file__), 'COS_conf.txt'))
		line = config.readline()
		line = line.rstrip()
		config.close()

		return line

	def establish_connection(self):
		port = 5005
		try:
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			s.connect((self.address,port))
		except:
			rs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			rs.connect((self.address,57011))
			print("Connection refused on " + self.address)
		return s

	'''
	COMMAND HANDLERS
	'''

	def handle_readAll(self, req):
		s = self.establish_connection()

		s.send("CS_Rall")
		data = s.recv(3000)

		s.shutdown(socket.SHUT_RDWR)
		s.close()

		data = str(data.decode())
		data = data.translate(None,"[]'")
		data = data.split(', ')
		data = filter(None, data)

		data_list = []
		step = 0

		for line in data:
			line = line.split(' ')
			step = len(line)
			for item in line:
				item = int(item)
				data_list.append(item)
		
		resp = COSReadAllResponse()
		resp.step = step
		resp.data = data_list

		return resp

	def handle_readOne(self, req):
		num = req.num.zfill(3)

		s = self.establish_connection()
		
		s.send('CS_Read'+num)
		data = s.recv(23)

		s.shutdown(socket.SHUT_RDWR)
		s.close()

		data = str(data.decode())
		data = data.translate(None,"[]'")
		# data = data.split(', ')
		data = filter(None, data)

		data_list = []

		for line in data:
			line = line.split(' ')
			for item in line:
				try:
					item = int(item)
					data_list.append(item)
				except:
					pass
					
		resp = COSReadOneResponse()
		resp.data = data_list

		return resp

	def handle_inteTime(self, req):
		num = min(max(req.num, 1), 250)

		s = self.establish_connection()
		
		s.send('CS_Inte'+num)
		a = s.recv(1024)

		s.shutdown(socket.SHUT_RDWR)
		s.close()
		
	'''
	HELPER FUNCTIONS
	'''
	
	def COS_server_init(self):
		rospy.init_node("COS_server")

		readAll_service = rospy.Service(
			"read_all",
			COSReadAll,
			self.handle_readAll)

		readOne_service = rospy.Service(
			"read_one",
			COSReadOne,
			self.handle_readOne)

		inteTime_service = rospy.Service(
			"inte_time",
			COSInteTime,
			self.handle_inteTime)

		print("Color sensor server ready")
		rospy.spin()

if (__name__ == "__main__"):
	ColorSensorServer()