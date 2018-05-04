#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import socket
from scr_control.srv import *
from scr_control.msg import *

def readAll_client():
	rospy.wait_for_service("readAll")
	try:
		readAll = rospy.ServiceProxy("readAll",COSReadAll)
		response = readAll()
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def readOne_client(num):
	rospy.wait_for_service('readOne')
	try:
		readOne = rospy.ServiceProxy("readOne",COSReadOne)
		response = readOne(num)
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def inteTime_client(num):
	rospy.wait_for_service('inteTime')
	try:
		inteTime = rospy.ServiceProxy("inteTime",COSInteTime)
		response = inteTime(num)
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def usage():
	print("%s read_all"%sys.argv[0])
	print("%s read [sensor_num]"%sys.argv[0])
	print("%s integration_time [time]"%sys.argv[0])
	print("%s help"%sys.argv[0])

if __name__ == '__main__':
	arg1 = str(sys.argv[1])
	arg1 = arg1.lower()
	if (arg1 == "read_all" and len(sys.argv) == 2):
		resp = readAll_client()
		# print(resp.step)
		for i in range(len(resp.data)):
			if(i%resp.step == 0 and i > 1):
				print()
			print(resp.data[i],end=" ")
	elif (arg1 == "read" and len(sys.argv) == 3):
		arg2 = sys.argv[2]
		resp = readOne_client(arg2)
		print(resp)

	elif (arg1 == "inte_time" and len(sys.argv) == 3):
		arg2 = sys.argv[2]
		resp = inteTime_client(arg2)
		print(resp)

	elif (arg1 == "help" and len(sys.argv) == 2):
		print()
		print("read_all: reads data from all sensors and prints values read")
		print()
		print("read: reads data from one sensor and prints values read")
		print("      first argument is the number of the sensor to read, ex. 1 or 23")
		print()
		print("inte_time: sets the integration time for the sensors")
		print("           first argument is the time in milliseconds to set integration time to")
		print()
		print("help: prints a list of all commands, what they do, and what arguments they take")
		print()

	else:
		usage()


