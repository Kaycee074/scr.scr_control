#!/usr/bin/env python

from scr_control.srv import *
import rospy
import socket
import time
import sys
import os
import re
import atexit

global address
global s


class HVACServer():

	'''
	INIT FUNCTIONS
	'''

	def __init__(self):
		self.address = self.initialize_hvac()
		self.s = self.establish_connection(60606)
		atexit.register(self.close)
		self.server_init()

	def initialize_hvac(self):
		config = open(os.path.join(os.path.dirname(__file__), 'SCR_HVAC_conf.txt'), 'r')
		line = config.readline()
		line = line.rstrip()
		address = line
		config.close()
		return address

	def establish_connection(self, port):
		try:
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			s.connect((self.address, port))
		except ConnectionRefusedError:
			# rs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			# rs.connect((address,57011))
			print("Connection refused on " + self.address)
		return s

	'''
	COMMAND HANDLERS
	'''

	def handle_setTemp(self, req):
		message = "set temp " + str(req.temp)
		self.send_message(message)
		resp = HVAC_SetTempResponse()
		return resp

	def handle_setFanSp(self, req):
		message = "set fan " + str(req.speed)
		self.send_message(message)
		resp = HVAC_SetFanSpResponse()
		return resp

	def handle_setEp(self, req):
		voltage = (5.0)*req.val/100
		message = "set ep" + str(req.ep) + " " + str(voltage)
		self.send_message(message)
		resp = HVAC_SetEpResponse()
		return resp

	def handle_setBms(self, req):
		self.send_message("bms")
		resp = HVAC_SetBmsResponse()
		return resp

	def handle_getTemp(self, req):
		temp_list = []
		for i in range(1,6):
			data = self.send_message("read t"+str(i))
			temp = [float(a) for a in re.findall("\d+\.\d+", data)]
			temp_list.append(temp[0])

		resp = HVAC_GetTempResponse()
		resp.data = temp_list

		return resp

	def handle_getEp(self, req):
		ep_list = []
		for i in range(1,5):
			data = self.send_message("read ep"+str(i))
			ep = [float(a) for a in re.findall("\d+\.\d+", data)]
			if i != 4:
				level = 100 - ep[0]
			else:
				level = ep[0]
			ep_list.append(level)

		resp = HVAC_GetEpResponse()
		resp.data = ep_list

		return resp

	def handle_getCO2(self, req):
		data = self.send_message("read co2")
		co2 = [float(a) for a in re.findall("\d+\.\d+", data)]
		resp = HVAC_GetCO2Response()
		resp.data = co2[0]
		return resp

	def handle_getRH(self, req):
		data = self.send_message("read rh")
		hum = [float(a) for a in re.findall("\d+\.\d+", data)]
		resp = HVAC_GetRHResponse()
		resp.data = hum[0]

		return resp

	'''
	HELPER FUNCTIONS
	'''
	def send_message(self, message):
		self.s.send(message.encode())	#send the message (thermostat value)
		data = self.s.recv(1024).decode()
		print ('Received from server: ' + data)
		return data

	def close(self):
		self.send_message("bms")
		self.send_message("quit")
		self.s.shutdown(socket.SHUT_RDWR)
		self.s.close()

	def server_init(self):

		rospy.init_node("HVAC_server")

		setTemp_service = rospy.Service(
			"set_temp",
			HVAC_SetTemp,
			self.handle_setTemp)

		setFanSp_service = rospy.Service(
			"set_fansp",
			HVAC_SetFanSp,
			self.handle_setFanSp)

		setEp_service = rospy.Service(
			"set_ep",
			HVAC_SetEp,
			self.handle_setEp)

		setBms_service = rospy.Service(
			"set_bms",
			HVAC_SetBms,
			self.handle_setBms)

		getTemp_service = rospy.Service(
			"get_temp",
			HVAC_GetTemp,
			self.handle_getTemp)

		getEp_service = rospy.Service(
			"get_ep",
			HVAC_GetEp,
			self.handle_getEp)

		getCO2_service = rospy.Service(
			"get_co2",
			HVAC_GetCO2,
			self.handle_getCO2)

		getRH_service = rospy.Service(
			"get_rh",
			HVAC_GetRH,
			self.handle_getRH)

		rospy.spin()

if (__name__ == "__main__"):
	HVACServer()