#!/usr/bin/env python

import sys
import rospy
import socket
from scr_control.srv import *
from scr_control.msg import *

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
	print("%s CCT [x_coord] [y_coord] [CCT] [intensity]"%sys.argv[0])
	print("%s ragbw [x_coord] [y_coord] [red] [amber] [green] [blue] [white]"%sys.argv[0])
	print("%s get_CCT [x_coord] [y_coord]"%sys.argv[0])
	print("%s get_int [x_coord] [y_coord]"%sys.argv[0])
	print("%s help"%sys.argv[0])

if(__name__ == "__main__"):
	arg1 = str(sys.argv[1])
	arg1 = arg1.lower()
	if (arg1 == "cct" and len(sys.argv) == 6):
		CCT_val = int(sys.argv[4])
		intensity_val = int(sys.argv[5])
		x = int(sys.argv[2])
		y = int(sys.argv[3])

		print("Changing light (%s,%s) to CCT %s and intensity %s"%(x,y,CCT_val,intensity_val))

		state = CCT_client(CCT_val,intensity_val,x,y)

		print("Light (%s,%s) is now in state %s"%(x,y,state))

	elif (arg1 == "ragbw" and len(sys.argv) == 9):
		red_val = int(sys.argv[4])
		amber_val = int(sys.argv[5])
		green_val = int(sys.argv[6])
		blue_val = int(sys.argv[7])
		white_val = int(sys.argv[8])
		x = int(sys.argv[2])
		y = int(sys.argv[3])

		print("Changing light (%s,%s) to red:%s%% amber:%s%% green:%s%% blue:%s%% white:%s%%"%(x,y,red_val,amber_val,green_val,blue_val,white_val))

		state = ragbw_client(red_val,amber_val,green_val,blue_val,white_val,x,y)

		print("Light (%s,%s) is now in state %s"%(x,y,state))

	elif arg1 == "get_cct" and len(sys.argv) == 4:
		x = int(sys.argv[2])
		y = int(sys.argv[3])
		state = get_CCT_client(x,y)
		print("The CCT of Light (%s,%s) is %s"%(x,y,state))

	elif arg1 == "get_int" and len(sys.argv) == 4:
		x = int(sys.argv[2])
		y = int(sys.argv[3])
		state = get_int_client(x,y)
		print("The intensity of Light (%s,%s) is %s"%(x,y,state))


	elif (arg1 == "help" and len(sys.argv) == 2):
		print()
		print("CCT: changes the color temperature and intensity of the light at the specified position")
		print("     first argument is the x coordinate of the light to change")
		print("     second argument is the y coordinate of the light to change")
		print("     third argument is a CCT value between 1800 and 10000")
		print("     fourth argument is an intensity value as a % betweem 0 and 100")
		print()
		print("ragbw: changes the color of the light at the specified postion")
		print("       first argument is the x coordinate of the light to change")
		print("       second argument is the y coordinate of the light to change")
		print("       third argument is a red color value as a % between 0 and 100")
		print("       fourth argument is a amber color value as a % between 0 and 100")
		print("       fifth argument is a green color value as a % between 0 and 100")
		print("       sixth argument is a blue color value as a % between 0 and 100")
		print("       seventh argument is a white color value as a % between 0 and 100")
		print()
		print("get_cct: returns the cct value of the light at the specified position if it has been changed")
		print("         first argument is the x coordinate of the light to change")
		print("         second argument is the y coordinate of the light to change")
		print()
		print("get_int: returns the intensity value of the light at the specified position if it has been changed")
		print("         first argument is the x coordinate of the light to change")
		print("         second argument is the y coordinate of the light to change")
		print()
		print("help: prints a list of all commands, what they do, and what arguments they take")
		print()

	else:
		usage()

	else:
		usage()
