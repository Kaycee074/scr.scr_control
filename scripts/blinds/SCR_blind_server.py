#!/usr/bin/env python

from scr_control.srv import *
import rospy
import socket
import time
import sys
import os

class BlindServer():

	'''
	INIT FUNCTIONS
	'''

	def __init__(self):
		self.blinds, self.address = self.initialize_blinds()
		self.s = socket.socket()
		self.handshake()
		self.server_init()

	def initialize_blinds(self):

		config = open(os.path.join(os.path.dirname(__file__), 'SCR_blind_conf.txt'), 'r')
		address = config.readline()
		config.readline()

		blinds = {}
		for line in config:
			line = line.rstrip()
			blinds[line] = len(blinds) + 1

		return blinds, address

	def handshake(self):
		self.s.connect((self.address, 23))
		self.s.send(b"nwk\r\n")
		print("Response: " + self.s.recv(1024).decode())

	'''
	COMMAND HANDLERS
	'''

	def handle_lift(self, req):
		val = self.change_blinds(req.blind, req.val, ",24,")
		return BlindLiftResponse(val)
		
	def handle_liftAll(self, req):
		for blind in self.blinds.keys():
			val = self.change_blinds(blind, req.val, ",24,")
		return BlindLiftAllResponse(val)

	def handle_tilt(self, req):
		val = self.change_blinds(req.blind, req.val, ",25,")
		return BlindLiftResponse(val)

	def handle_tiltAll(self, req):
		for blind in self.blinds.keys():
			val = self.change_blinds(blind, req.val, ",25,")
		return BlindTiltAllResponse(val)

	def handle_getBlinds(self, req):
		val = self.blinds.keys()
		return GetBlindsResponse(val)
		
	'''
	HELPER FUNCTIONS
	'''
	
	def change_blinds(self, blind, val, action):

		val = min(max(0, val), 100)

		if blind not in self.blinds:
			print("Blind %s is not a valid blind." % b)
			sys.exit(1)

		cmdstr = "#DEVICE,QS," + str(self.blinds[blind]) + action + str(val) + "\r\n"

		self.s.send(cmdstr)
		a = self.s.recv(2048)

		return val

	def server_init(self):
		rospy.init_node("blind_server")

		lift_service = rospy.Service(
			"lift",
			BlindLift,
			self.handle_lift)

		liftAll_service = rospy.Service(
			"lift_all",
			BlindLiftAll,
			self.handle_liftAll)

		tilt_service = rospy.Service(
			"tilt",
			BlindTilt,
			self.handle_tilt)

		tilt_service = rospy.Service(
			"tilt_all",
			BlindTiltAll,
			self.handle_tiltAll)

		getBlinds_service = rospy.Service(
			"get_blinds",
			GetBlinds,
			self.handle_getBlinds)

		rospy.spin()

if (__name__ == "__main__"):
	BlindServer()