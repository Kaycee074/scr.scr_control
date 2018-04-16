#!/usr/bin/env python

from light.srv import *
import rospy
import socket
import time
import sys
import os

global address


def initialize():
	global address
	__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

	config = open(os.path.join(__location__,"COS_conf.txt"),'r')

	line = config.readline()
	line = line.rstrip()

	address = line

	config.close()

def establish_connection(address):
	port = 5005
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((address,port))
	except ConnectionRefusedError:
		rs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		rs.connect((address,57011))
		print("Connection refused on " + address)
	return s


def handle_readAll(req):
	s = establish_connection(address)

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

def handle_readOne(req):
	num = req.num.zfill(3)

	s = establish_connection(address)
	
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
			item = int(item)
			data_list.append(item)

	resp = COSReadOneResponse()
	resp.data = data_list

	return resp

def handle_inteTime(req):
	num = req.num

	s = establish_connection(address)
	
	s.send('CS_Inte'+num)
	a = s.recv(1024)

	s.shutdown(socket.SHUT_RDWR)
	s.close()

def COS_server():
	rospy.init_node("COS_server")

	readAll_service = rospy.Service(
		"readAll",
		COSReadAll,
		handle_readAll)

	readOne_service = rospy.Service(
		"readOne",
		COSReadOne,
		handle_readOne)

	rospy.spin()

if (__name__ == "__main__"):
	
	initialize()
	
	COS_server()