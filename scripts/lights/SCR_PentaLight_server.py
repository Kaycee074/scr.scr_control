#!/usr/bin/env python

from scr_control.srv import *
import rospy
import socket
import time
import sys
import os

class PentaLightServer():

	def __init__(self):

		self.lights = self.initialize_lights()

		self.CCT_int_memory = {}
		for key in self.lights:
			self.CCT_int_memory[key] = [0, 0]

		self.CCT_dict = self.initialize_CCT()
		self.int_list = self.initialize_int()

		self.serverInit()

	def initialize_lights(self):
		__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

		config = open(os.path.join(__location__,"SCR_PentaLight_conf.txt"),'r')
		lights = []
		addresses = []
		position_array = []

		# read addresses
		for line in config:
			line = line.rstrip()
			if line == '-': break
			addresses.append(line)

		# read position array
		first = True
		for line in config:
			line = line.rstrip()
			# read the first line seperately to create initial lists
			if first:
				for char in line:
						col = []
						col.append(int(char))
						position_array.append(col)
				first = False
			else:
				i = 0
				for char in line:
					position_array[i].append(int(char))
					i += 1
		
		config.close()

		# get light coordinates from array
		light_coords = []
		for i in range(len(position_array)):
			for j in range(len(position_array[i])):
				if position_array[i][j] == 1:
					light_coords.append((i,j))
		
		# print light_coords
		if len(light_coords) != len(addresses):
			print ("Error: There are %s addresses and %s lights in the array"%(len(addresses),len(light_coords)))
			sys.exit(1)
		
		# create dictionary with coord as key and address as value
		lights = {}
		for i in range(len(light_coords)):
			lights[light_coords[i]] = addresses[i]

		return lights

	def initialize_CCT(self):
		__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

		vals = open(os.path.join(__location__,"SCR_PentaLight_CCT.txt"),'r')
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

		# ragbw
		return CCT_dict

	def initialize_int(self):
		__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

		vals = open(os.path.join(__location__,"SCR_PentaLight_int.txt"),"r")
		ints = []
		for line in vals:
			line = line.rstrip()
			split = line.split("\t")
			for i  in range(len(split)):
				split[i] = float(split[i])
			ints.append(split)
		vals.close()
		# print(ints)
		# bgarw
		return ints

	def gen_cmdstr_CCT(self, state, intensity):
		# blue = green = amber = red = white  = "0000"

		send_blue = self.CCT_dict[state][3]
		send_green = self.CCT_dict[state][2]
		send_amber = self.CCT_dict[state][1]
		send_red = self.CCT_dict[state][0]
		send_white = self.CCT_dict[state][4]

		intensity = float(intensity)/100

		intens_b = int_list[0][0]*intensity*intensity+self.int_list[0][1]*intensity
		intens_g = int_list[1][0]*intensity*intensity+self.int_list[1][1]*intensity
		intens_a = int_list[2][0]*intensity*intensity+self.int_list[2][1]*intensity
		intens_r = int_list[3][0]*intensity*intensity+self.int_list[3][1]*intensity
		intens_w = int_list[4][0]*intensity*intensity+self.int_list[4][1]*intensity

		blue = int(65535*send_blue*intens_b)
		green = int(65535*send_green*intens_g)
		amber = int(65535*send_amber*intens_a)
		red = int(65535*send_red*intens_r)
		white = int(65535*send_white*intens_w)

		return self.gen_cmdstr_modified_ragbw(blue, green, amber, red, white)

	def gen_cmdstr_ragbw(self, req):
		blue = int(65535*(req.blue/100))
		green = int(65535*(req.green/100))
		amber = int(65535*(req.amber/100))
		red = int(65535*(req.red/100))
		white = int(65535*(req.white/100))

		return self.gen_cmdstr_modified_ragbw(blue, green, amber, red, white)

	def gen_cmdstr_modified_ragbw(self, blue, green, amber, red, white):
		
		blue = format(blue,'x')
		green = format(green,'x')
		amber = format(amber,'x')
		red = format(red,'x')
		white = format(white,'x')

		blue = blue.rjust(4,'0')
		green = green.rjust(4,'0')
		amber = amber.rjust(4,'0')
		red = red.rjust(4,'0')
		white = white.rjust(4,'0')

		return "PS"+blue+green+amber+red+white

	def handle_CCT(self, req):

		intensity = max(min(req.intensity, 100), 0)
		CCT = max(min(req.CCT, 10000), 1800)
		if CCT != 10000 and CCT != 1800:
			CCT = req.CCT - req.CCT%100

		cmdstr = self.gen_cmdstr_CCT(CCT, intensity)
		self.change_light(req.x, req.y, cmdstr)

		self.CCT_int_memory[(req.x,req.y)][0] = CCT
		self.CCT_int_memory[(req.x,req.y)][1] = intensity

		return PentaLight_CCTResponse(cmdstr)

	# def handle_int(req,lights):
	# return
	# values other than 100 dont work
	def handle_ragbw(self, req):

		cmdstr = self.gen_cmdstr_ragbw(req)
		self.change_light(req.x, req.y, cmdstr)

		return PentaLight_ragbwResponse(cmdstr)

	def change_light(self, x, y, cmdstr):
		
		port = 57007

		if not (x, y) in self.lights:
			return "Error: No light at specified coordinates"

		address = self.lights[(x, y)]

		try:
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			s.connect((address,port))
		except ConnectionRefusedError:
			rs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			rs.connect((address,57011))
			print("Connection refused")

		s.send(cmdstr)
		a = s.recv(1024)
		time.sleep(0.001)

		s.shutdown(socket.SHUT_RDWR)
		s.close()

	def handle_getCCT(self, req):
		out = str(self.CCT_int_memory[(req.x,req.y)][0])
		return GetCCTResponse(out);

	def handle_getInt(self, req):
		out = CCT_int_memory[(req.x,req.y)][1]
		return GetIntResponse(out)

	def serverInit(self):
		rospy.init_node("PentaLight_server")

		CCT_service = rospy.Service(
			"CCT", 
			PentaLight_CCT, 
			lambda msg: self.handle_CCT(msg))

		ragbw_service = rospy.Service(
			"ragbw", 
			PentaLight_ragbw, 
			lambda msg: self.handle_ragbw(msg))

		get_CCT_service = rospy.Service(
			"getCCT",
			GetCCT,
			lambda msg: self.handle_getCCT(msg))

		get_int_service = rospy.Service(
			"getInt",
			GetInt,
			lambda msg: self.handle_getInt(msg))

		print("PentaLight server ready")
		rospy.spin()

if (__name__ == "__main__"):
	
	PentaLightServer()