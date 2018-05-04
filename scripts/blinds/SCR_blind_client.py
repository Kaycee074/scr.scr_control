#!/usr/bin/env python

import sys
import rospy
import socket
from scr_control.srv import *
from scr_control.msg import *

def lift_client(b,val):
	rospy.wait_for_service('lift')
	try:
		lift = rospy.ServiceProxy('lift',BlindLift)
		state = lift(b,val)
		return state
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def tilt_client(b,val):
	rospy.wait_for_service('tilt')
	try:
		tilt = rospy.ServiceProxy('tilt',BlindTilt)
		state = tilt(b,val)
		return state
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def usage():
	print("%s lift [blind] [percent]"%sys.argv[0])
	print("%s tilt [blind] [percent]"%sys.argv[0])
	print("%s help"%sys.argv[0])

if(__name__ == "__main__"):
	arg1 = str(sys.argv[1])
	arg1 = arg1.lower()
	if (arg1 == "lift" and len(sys.argv) == 4):
		blind = sys.argv[2]
		val = int(sys.argv[3])
	
		state = lift_client(blind,val)
		print("Blind %s is now lifted to %s%%"%(blind,state))

	elif (arg1 == 'tilt' and len(sys.argv) == 4):
		blind = sys.argv[2]
		val = int(sys.argv[3])
		state = tilt_client(blind,val)
		print("Blind %s is now tilted to %s%%"%(blind,state))

	elif (arg1 == 'help'):
		print()
		print("lift: lifts the specified blind to the input percent")
		print("      first argument is a blind orientation+number, ex. N3 or S1")
		print("      second argument is a percent to raise the blind to")
		print()
		print("tilt: tilts the slats of a specified blinf to the input percent")
		print("      first argument is a blind orientation+number, ex. N3 or S1")
		print("      second argument is a percent to tilt the slats to")
		print()
		print("help: prints a list of all commands, what they do, and what arguments they take")
		print()

	else:
		usage()