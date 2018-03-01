#!/usr/bin/env python

from light.srv import *
import rospy
import socket
import time
import sys

def initialize():
	config = open("SCR_PentaLight_conf.txt",'r')
	lights = []
	addresses = []
	position_array = []

	#read addresses
	for line in config:
		line = line.rstrip()
		if line == '-': break
		addresses.append(line)

	#read position array
	first = True
	for line in config:
		line = line.rstrip()
		#read the first line seperately to create initial lists
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

	#get light coordinates from array
	light_coords = []
	for i in range(len(position_array)):
		for j in range(len(position_array[i])):
			if position_array[i][j] == 1:
				light_coords.append((i,j))
	print light_coords

	if len(light_coords) != len(addresses):
		print ("Error: There are %s addresses and %s lights in the array"%(len(addresses),len(light_coords)))
		sys.exit(1)
	
	#create dictionary with coord as key and address as value
	lights = {}
	for i in range(len(light_coords)):
		lights[light_coords[i]] = addresses[i]

	return lights

def initialize_CCT():
	vals = open("SCR_PentaLight_CCT.txt",'r')
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

def initialize_int():
	vals = open("SCR_PentaLight_int.txt","r")
	ints = []
	for line in vals:
		line = line.rstrip()
		split = line.split("\t")
		ints.append[line]
	vals.close()

	print ints
	return ints


def handle_CCT(req,lights,CCT_dict):
	return

def handle_int(req,lights):
	return

def handle_ragbw(req,lights):
	return

def PentaLight_server(lights,CCT_dict):
	rospy.init_node("PentaLight_server")

	CCT_service = rospy.Service(
		"CCT", 
		PentaLight_CCT, 
		lambda msg: handle_CCT(msg,lights,CCT_dict))

	intensity_service = rospy.Service(
		"intensity", 
		PentaLight_int, 
		lambda msg: handle_int(msg,lights))

	ragbw_service = rospy.Service(
		"ragbw", 
		PentaLight_ragbw, 
		lambda msg: handle_ragbw(msg,lights))

if (__name__ == "__main__"):
	lights = initialize()
	CCT_dict = initialize_CCT()
	int_list = initialize_int()
	# PentaLight_server(lights,CCT_dict)


