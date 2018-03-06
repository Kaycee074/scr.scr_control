#!/usr/bin/env python

from light.srv import *
import rospy
import socket
import time
import sys
import os

global blinds
global address
global s

blinds = {}
address = ""
#s = ''

def initialize():
	global blinds
	global address

	__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

	config = open(os.path.join(__location__,"SCR_blind_conf.txt"),'r')
	first = True

	i = 1
	for line in config:
		line = line.rstrip()
		if first:
			address = line
			first = False
		elif line == '-':
			pass
		else:
			blinds[line] = i
			i += 1

def handshake(s):
	global address
	s.connect((address, 23))
	s.send(b"nwk\r\n")
	print("Response: " + s.recv(1024).decode())

def handle_lift(req):
	global s
	global blinds

	b = req.blind
	val = req.val

	if val < 0:
		val = 0
	if val > 100:
		val = 100

	if b not in blinds:
		print("Blind %s is not a valid blind."%b)
		sys.exit(1)

	cmdstr = "#DEVICE,QS," + str(blinds[b]) + ",24," + str(val) + "\r\n"
	#cmdstr = bytes(cmdstr, "UTF-8")

	s.send(cmdstr)
	a = s.recv(2048)

	return BlindLiftResponse(val)

def handle_tilt(req):
	global s
	global blinds

	b = req.blind
	val = req.val

	if val < 0:
		val = 0
	if val > 100:
		val = 100

	if b not in blinds:
		print("Blind %s is not a valid blind."%b)
		sys.exit(1)

	cmdstr = "#DEVICE,QS," + str(blinds[b]) + ",25," + str(val) + "\r\n"
	#cmdstr = bytes(cmdstr, "UTF-8")

	s.send(cmdstr)
	a = s.recv(2048)
	return 

def blind_server():
	rospy.init_node("blind_server")

	lift_service = rospy.Service(
		"lift",
		BlindLift,
		handle_lift)

	tilt_service = rospy.Service(
		"tilt",
		BlindTilt,
		handle_tilt)
	rospy.spin()


if (__name__ == "__main__"):
	global s

	initialize()

	s = socket.socket()
	handshake(s)

	blind_server()