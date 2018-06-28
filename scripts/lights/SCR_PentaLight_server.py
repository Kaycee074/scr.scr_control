#!/usr/bin/env python

from scr_control.srv import *
import rospy
import socket
import time
import sys
import os

class PentaLightServer():

	'''
	INIT FUNCTIONS
	'''

	def __init__(self):

		self.lights = self.initialize_lights()

		self.CCT_int_memory = {}
		for key in self.lights:
			self.CCT_int_memory[key] = [0, 0]

		self.CCT_dict = self.initialize_CCT()
		self.int_list = self.initialize_int()

		self.server_init()

	def initialize_lights(self):

		config = open(os.path.join(os.path.dirname(__file__), 'SCR_PentaLight_conf.txt'), 'r')
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

	def initialize_CCT(self):

		vals = open(os.path.join(os.path.dirname(__file__), 'SCR_PentaLight_CCT.txt'), 'r')
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

	def initialize_int(self):

		vals = open(os.path.join(os.path.dirname(__file__), 'SCR_PentaLight_int.txt'), 'r')
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
	COMMAND HANDLERS
	'''

	def handle_CCT(self, req):
		intensity, CCT = self.getIntensityCCT(req.intensity, req.CCT)
		cmdstr = self.gen_cmdstr_CCT(CCT, intensity)
		self.change_light(req.x, req.y, cmdstr)
		self.CCT_int_memory[(req.x,req.y)][0] = CCT
		self.CCT_int_memory[(req.x,req.y)][1] = intensity

		return PentaLight_CCTResponse(cmdstr)

	def handle_ragbw(self, req):
		cmdstr = self.gen_cmdstr_ragbw(req)
		return self.change_light(req.x, req.y, cmdstr) or PentaLight_ragbwResponse(cmdstr)

	def handle_CCTAll(self, req):
		intensity, CCT = self.getIntensityCCT(req.intensity, req.CCT)
		cmdstr = self.gen_cmdstr_CCT(CCT, intensity)

		for light in self.lights.keys():
			self.CCT_int_memory[(light[0], light[1])][0] = CCT
			self.CCT_int_memory[(light[0], light[1])][1] = intensity
			self.change_light(light[0], light[1], cmdstr)

		return PentaLight_CCTAllResponse(cmdstr)

	def handle_ragbwAll(self, req):
		cmdstr = self.gen_cmdstr_ragbw(req)
		for light in self.lights.keys():
			x = light[0]
			y = light[1]
			self.change_light(x, y, cmdstr)
		return PentaLight_ragbwAllResponse(cmdstr)

	def handle_getCCT(self, req):
		out = str(self.CCT_int_memory[(req.x,req.y)][0])
		return GetCCTResponse(out);

	def handle_getInt(self, req):
		out = self.CCT_int_memory[(req.x,req.y)][1]
		return GetIntResponse(out)

	def handle_getLights(self, req):
		out = []
		for light in self.lights.keys():
			out.append(light[0])
			out.append(light[1])
		return GetLightsResponse(out)
		
	'''
	HELPER FUNCTIONS
	'''

	def gen_cmdstr_CCT(self, CCT, intensity):

		colors = []
		for i in range(5):
			colors.append(self.CCT_dict[CCT][i])

		intensity = float(intensity)/100
		intensity_colors = []
		for i in range(5):
			intensity_colors.append(self.int_list[i][0]*intensity*intensity+self.int_list[i][1]*intensity)
		
		for i in range(5):
			colors[i] = int(65535*colors[i]*intensity_colors[i])

		return self.gen_cmdstr_modified_ragbw(colors)

	def gen_cmdstr_ragbw(self, req):
		colors = [req.blue, req.green, req.amber, req.red, req.white]
		colors = [int(65535*(float(x)/100)) for x in colors]
		return self.gen_cmdstr_modified_ragbw(colors)

	def gen_cmdstr_modified_ragbw(self, colors):
		colors = [format(x, 'x') for x in colors]
		colors = [x.rjust(4, '0') for x in colors]
		return "PS"+''.join(colors)

	def getIntensityCCT(self, i, c):
		intensity = max(min(i, 100), 0)
		CCT = max(min(c, 10000), 1800)
		if CCT != 10000 and CCT != 1800:
			CCT = c - c%100
		return intensity, CCT

	def change_light(self, x, y, cmdstr):
		
		if not (x, y) in self.lights:
			return "Error: No light at specified coordinates"

		port = 57007
		address = self.lights[(x, y)]

		#start = time.time()

		try:
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			s.connect((address,port))

		except:
			rs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			rs.connect((address, 57011))
			print("Connection refused")

		s.send(cmdstr)
		a = s.recv(1024)
		time.sleep(0.001)

		s.shutdown(socket.SHUT_RDWR)
		s.close()

		#end = time.time()

		#print("changed (" + str(x) + ", " + str(y) + ") to cmdstr: " + str(cmdstr) + " in " + str(end-start) + " seconds")

		return None

	def server_init(self):
		rospy.init_node("PentaLight_server")

		CCT_service = rospy.Service(
			"cct", 
			PentaLight_CCT, 
			lambda msg: self.handle_CCT(msg))

		ragbw_service = rospy.Service(
			"ragbw", 
			PentaLight_ragbw, 
			lambda msg: self.handle_ragbw(msg))

		CCTAll_service = rospy.Service(
			"cct_all", 
			PentaLight_CCTAll, 
			lambda msg: self.handle_CCTAll(msg))

		ragbwAll_service = rospy.Service(
			"ragbw_all", 
			PentaLight_ragbwAll, 
			lambda msg: self.handle_ragbwAll(msg))

		get_CCT_service = rospy.Service(
			"get_cct",
			GetCCT,
			lambda msg: self.handle_getCCT(msg))

		get_int_service = rospy.Service(
			"get_int",
			GetInt,
			lambda msg: self.handle_getInt(msg))

		get_lights_service = rospy.Service(
			"get_lights",
			GetLights,
			lambda msg: self.handle_getLights(msg))

		print("PentaLight server ready")
		rospy.spin()

if (__name__ == "__main__"):
	PentaLightServer()