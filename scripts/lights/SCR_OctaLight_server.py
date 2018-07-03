#!/usr/bin/env python

from scr_control.srv import *
import rospy
import socket
import time
import sys
import os
import api

class OctaLightServer():

	'''
	INIT FUNCTIONS
	'''

	def __init__(self):
		self.lights = self.read_config()
		self.CCT_dict = self.initialize_CCT()
		self.ip_list = self.initialize_lights()
		self.server_init()

	def read_config(self):

		config = open(os.path.join(os.path.dirname(__file__), 'SCR_OctaLight_conf.txt'), 'r')
		lights = {}
		addresses = []

		# read addresses
		for line in config:
			line = line.rstrip()
			if line == '-': break
			addresses.append(line)

		# read position array
		y = 0
		for line in config:
			line = line.rstrip()
			x = 0
			for char in line:
				if char == '1':
					if len(lights) > len(addresses):
						print ("Error: There are %s addresses and %s lights in the array" % (len(addresses), len(light_coords)))
						sys.exit(1)
					lights[(x, y)] = addresses[len(lights)]
				x += 1
			y += 1

		config.close()

		return lights

	def initialize_lights(self):
		sys.stderr = open(os.devnull, "w")
		ip_list = api.discover()
		sys.stdout = sys.__stdout__
		return ip_list

	def initialize_CCT(self):

		vals = open(os.path.join(os.path.dirname(__file__), 'SCR_OctaLight_CCT.txt'), 'r')
		CCT_vals = []
		light_vals = []

		first_line = True
		for line in vals:
			if first_line:
				first_line = False
			else:
				line = line.rstrip();
				split = line.split("\t")
				first = True
				temp_light_vals = []
				for el in split:
					if first:
						CCT_vals.append(int(el))
						first = False
					else:
						temp_light_vals.append(float(el))
				light_vals.append(temp_light_vals)

		vals.close()
		
		CCT_dict = {}
		for i in range(len(CCT_vals)):
			CCT_dict[CCT_vals[i]] = light_vals[i]

		return CCT_dict

	'''
	def initialize_int(self):

		vals = open(os.path.join(os.path.dirname(__file__), 'SCR_OctaLight_int.txt'), 'r')
		ints = []
		for line in vals:
			line = line.rstrip()
			split = line.split("\t")
			for i  in range(len(split)):
				split[i] = float(split[i])
			ints.append(split)
		vals.close()

		return ints
	'''

	'''
	COMMAND HANDLERS
	'''

	def handle_CCT(self, req):
		intensity, CCT = self.getIntensityCCT(req.intensity, req.CCT)
		channels = self.get_channels_CCT(CCT, intensity)
		self.change_light(req.x, req.y, channels)
		return OctaLight_CCTResponse(str(channels))

	def handle_sources(self, req):
		channels = self.get_channels(req)
		self.change_light(req.x, req.y, channels)
		return OctaLight_sourcesResponse(str(channels))

	def handle_CCTAll(self, req):
		intensity, CCT = self.getIntensityCCT(req.intensity, req.CCT)
		channels = self.get_channels_CCT(CCT, intensity)
		self.change_light_all(self.gen_cmdstr(channels))
		return OctaLight_CCTAllResponse(str(channels))

	def handle_sourcesAll(self, req):
		channels = self.get_channels(req)
		self.change_light_all(self.gen_cmdstr(channels))
		return OctaLight_sourcesAllResponse(str(channels))

	def handle_getSources(self, req):
		if not (req.x, req.y) in self.lights:
			return "Error: No light at specified coordinates"
		address = self.lights[(req.x, req.y)]
		channels = api.get_all_drive_levels(address)
		channels = [float(x*100) for x in channels]
		return GetSourcesResponse(channels);

	def handle_getLights(self, req):
		out = []
		for light in self.lights.keys():
			out.append(light[0])
			out.append(light[1])
		return GetLightsResponse(out)

	'''
	HELPER FUNCTIONS
	'''

	def get_channels(self, req):
		colors = [req.b1, req.b2, req.b3, req.l, req.a, req.o, req.r1, req.r2]
		for i in range(len(colors)):
			colors[i] = float(float(colors[i])/100)
		return colors

	def get_channels_CCT(self, CCT, intensity):
		colors = []
		for i in range(8):
			colors.append(float(float(self.CCT_dict[CCT][i])/100)*intensity)
		return colors

	def gen_cmdstr(self, colors):
		colors = [int(x*65535) for x in colors]
		colors = [format(x, 'x') for x in colors]
		colors = [x.rjust(4, '0') for x in colors]
		return "PS"+''.join(colors)

	def getIntensityCCT(self, i, c):
		intensity = float(max(min(i, 100), 0))/100
		CCT = max(min(c, 11500), 1500)
		CCT = CCT - c%500
		if (CCT%1000 == 0):
			CCT += 500
		return intensity, CCT

	def change_light_all(self, cmdstr):
		api.sendMessageParallel(self.ip_list, cmdstr)
		return "Lights changed to " + cmdstr

	def change_light(self, x, y, channels):
		if not (x, y) in self.lights:
			return "Error: No light at specified coordinates"
		address = self.lights[(x, y)]
		api.set_all_drive_levels(address, channels)
		return str(channels)

	def server_init(self):
		rospy.init_node("OctaLight_server")

		CCT_service = rospy.Service(
			"cct", 
			OctaLight_CCT, 
			lambda msg: self.handle_CCT(msg))

		sources_service = rospy.Service(
			"sources", 
			OctaLight_sources, 
			lambda msg: self.handle_sources(msg))

		CCTAll_service = rospy.Service(
			"cct_all", 
			OctaLight_CCTAll, 
			lambda msg: self.handle_CCTAll(msg))

		sourcesAll_service = rospy.Service(
			"sources_all", 
			OctaLight_sourcesAll, 
			lambda msg: self.handle_sourcesAll(msg))

		get_Sources_service = rospy.Service(
			"get_sources",
			GetSources,
			lambda msg: self.handle_getSources(msg))

		get_lights_service = rospy.Service(
			"get_lights",
			GetLights,
			lambda msg: self.handle_getLights(msg))

		print("OctaLight server ready")
		rospy.spin()

if (__name__ == "__main__"):
	OctaLightServer()