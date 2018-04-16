#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import socket
from light.srv import *
from light.msg import *

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
	return

if __name__ == '__main__':
	if (len(sys.argv) == 2):
		command = sys.argv[1]
		if (command == 'readAll'):
			resp = readAll_client()
			# print(resp.step)
			for i in range(len(resp.data)):
				if(i%resp.step == 0 and i > 1):
					print()
				print(resp.data[i],end=" ")
	elif (len(sys.argv) == 3):
		command = sys.argv[1]
		arg2 = sys.argv[2]
		if (command == 'read'):
			resp = readOne_client(arg2)

		elif (command == 'integrate'):
			resp = inteTime_client(arg2)


