#!/usr/bin/env python

from light.srv import *
import rospy
import socket
import time
import sys
import os
import re
import atexit

global address
global s

def initialize():
	global address
	__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

	config = open(os.path.join(__location__,"COS_conf.txt"),'r')

	line = config.readline()
	line = line.rstrip()

	address = line

	config.close()

def establish_connection(address):
	global s
	port = 60606
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((address,port))
	except ConnectionRefusedError:
		# rs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		# rs.connect((address,57011))
		print("Connection refused on " + address)
	return s



def send_message(message):
	s.send(message.encode())	#send the message (thermostat value)
	data = s.recv(1024).decode()
	print ('Received from server: ' + data)
	return data

def close():
	send_message("quit")
	s.shutdown(socket.SHUT_RDWR)
	s.close()

def handle_setTemp(req):
	message = "set temp " + str(req.temp)
	send_message(message)

def handle_setFanSp(req):
	message = "set fan " + str(req.speed)
	send_message(message)

def handle_setEp(req):
	voltage = (5.0)*req.val/100
	message = "set ep" + str(req.ep) + " " + str(voltage)
	send_message(message)

def handle_setBms(req):
	send_message("bms")

def handle_getTemp(req):
	temp_list = []
	for i in range(5):
		data = send_message("read t"+str(i))
		temp = [float(s) for s in re.findall("\d+\.\d+", data)]
		temp_list.append(temp[0])

	resp = HVAC_GetTempResponse()
	resp.data = temp_list

	return resp

def handle_getEp(req):
	ep_list = []
	for i in range(1,5):
		data = send_message("read ep"+str(i))
		ep = [float(s) for s in re.findall("\d+\.\d+", data)]
		if i != 4:
			level = 100 - ep[0]
		else:
			level = ep[0]
		ep_list.append(level)

	resp = HVAC_GetEpResponse()
	resp.data = ep_list

	return resp

def handle_getCO2(req):
	data = send_message("read co2")
	co2 = [float(s) for s in re.findall("\d+\.\d+", data)]

	resp = HVAC_GetCO2Response()
	resp.data = co2[0]

	return resp

def handle_getRH(req):
	data = send_message("read co2")
	hum = [float(s) for s in re.findall("\d+\.\d+", data)]

	resp = HVAC_GetCO2Response()
	resp.data = hum[0]

	return resp

def HVAC_server():
	rospy.init_node("HVAC_server")

	setTemp_service = rospy.Service(
		"setTemp",
		HVAC_SetTemp,
		handle_setTemp)

	setFanSp_service = rospy.Service(
		"setFanSp",
		HVAC_SetFanSp,
		handle_setFanSp)

	setEp_service = rospy.Service(
		"setEp",
		HVAC_SetEp,
		handle_setEp)

	setBms_service = rospy.Service(
		"setBms",
		HVAC_SetBms,
		handle_setBms)

	getTemp_service = rospy.Service(
		"getTemp",
		HVAC_GetTemp,
		handle_getTemp)

	getEp_service = rospy.Service(
		"getEp",
		HVAC_GetEp,
		handle_getEp)

	getCO2_service = rospy.Service(
		"getCO2",
		HVAC_GetCO2,
		handle_getCO2)

	getRH_service = rospy.Service(
		"getRH",
		HVAC_GetRH,
		handle_getRH)

	rospy.spin()

if (__name__ == "__main__"):
	atexit.register(close)
	initialize();
	establish_connection();
	HVAC_server();