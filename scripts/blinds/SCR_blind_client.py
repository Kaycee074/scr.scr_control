#!/usr/bin/env python

import sys
import rospy
import socket
from light.srv import *
from light.msg import *

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
	return

if(__name__ == "__main__"):
	if (len(sys.argv) == 4):
		blind = sys.argv[2]
		val = int(sys.argv[3])
		if str(sys.argv[1]) == 'lift':
			state = lift_client(blind,val)
			print("Blind %s is now lifted to %s%%"%(blind,state))

		elif str(sys.argv[1]) == 'tilt':
			state = tilt_client(blind,val)
			print("Blind %s is now tilted to %s%%"%(blind,state))
	else:
		usage()