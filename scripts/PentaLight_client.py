#!/usr/bin/env python

import sys
import rospy
import socket
from light.srv import *
from light.msg import *

def CCT_client(CCT_val,intensity_val,x,y):
	rospy.wait_for_service('CCT')
	try:
		#					service name  .srv file name
		CCT = rospy.ServiceProxy('CCT',PentaLight_CCT)
		state = CCT(CCT_val,intensity_val,x,y)
		return state
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def ragbw_client(red_val,amber_val,green_val,blue_val,white_val,x,y):
	rospy.wait_for_service('ragbw')
	try:
		#					service name  .srv file name
		ragbw = rospy.ServiceProxy('ragbw',PentaLight_ragbw)
		state = ragbw(red_val,amber_val,green_val,blue_val,white_val,x,y)
		return state
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def get_CCT_client(x,y):
	rospy.wait_for_service('getCCT')
	try:
		get_CCT = rospy.ServiceProxy('getCCT',GetCCT)
		state = get_CCT(x,y)
		return state
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def get_int_client(x,y):
	rospy.wait_for_service('getInt')
	try:
		get_int = rospy.ServiceProxy('getInt',GetInt)
		state = get_int(x,y)
		return state
	except rospy.ServiceException, e:
				print("Service call failed: %s"%e)

def usage():
	print("%s CCT intensity x_coord y_coord"%sys.argv[0])
	print("%s red amber green blue white x_coord y_coord"%sys.argv[0])
	print("%s HELP"%sys.argv[0])

if(__name__ == "__main__"):
	if (len(sys.argv) == 5):
		CCT_val = int(sys.argv[1])
		intensity_val = int(sys.argv[2])
		x = int(sys.argv[3])
		y = int(sys.argv[4])

		print("Changing light (%s,%s) to CCT %s and intensity %s"%(x,y,CCT_val,intensity_val))

		state = CCT_client(CCT_val,intensity_val,x,y)

		print("Light (%s,%s) is now in state %s"%(x,y,state))

	elif (len(sys.argv) == 8):
		red_val = int(sys.argv[1])
		amber_val = int(sys.argv[2])
		green_val = int(sys.argv[3])
		blue_val = int(sys.argv[4])
		white_val = int(sys.argv[5])
		x = int(sys.argv[6])
		y = int(sys.argv[7])

		print("Changing light (%s,%s) to red:%s%% amber:%s%% green:%s%% blue:%s%% white:%s%%"%(x,y,red_val,amber_val,green_val,blue_val,white_val))

		state = ragbw_client(red_val,amber_val,green_val,blue_val,white_val,x,y)

		print("Light (%s,%s) is now in state %s"%(x,y,state))

	elif (len(sys.argv) == 4):
		arg = str(sys.argv[1])
		arg = arg.lower()

		x = int(sys.argv[2])
		y = int(sys.argv[3])

		if arg == "cct":
			state = get_CCT_client(x,y)
			print("The CCT of Light (%s,%s) is %s"%(x,y,state))

		if arg == "int":
			state = get_int_client(x,y)
			print("The intensity of Light (%s,%s) is %s"%(x,y,state))


	elif (len(sys.argv) == 2):
		arg = str(sys.argv[1])
		arg = arg.lower()

		if (arg == "help"):
			print("how to use the function")
		else:
			usage()

	else:
		usage()
