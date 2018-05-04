#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import socket
from scr_control.srv import *
from scr_control.msg import *

def setTemp_client(temp):
	rospy.wait_for_service("setTemp")
	try:
		setTemp = rospy.ServiceProxy("setTemp",HVAC_SetTemp)
		response = setTemp(temp)
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def setFanSp_client(speed):
	rospy.wait_for_service("setTemp")
	try:
		setFanSp = rospy.ServiceProxy("setFanSp",HVAC_SetFanSp)
		response = setFanSp(speed)
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def setEp_client(ep,val):
	rospy.wait_for_service("setTemp")
	try:
		setEp = rospy.ServiceProxy("setEp",HVAC_SetEp)
		response = setEp(ep,val)
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def setBms_client():
	rospy.wait_for_service("setTemp")
	try:
		setBms = rospy.ServiceProxy("setBms",HVAC_SetBms)
		response = setBms()
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def getTemp_client():
	rospy.wait_for_service("setTemp")
	try:
		getTemp = rospy.ServiceProxy("getTemp",HVAC_GetTemp)
		response = getTemp()
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def getEp_client():
	rospy.wait_for_service("setTemp")
	try:
		getEp = rospy.ServiceProxy("getEp",HVAC_GetEp)
		response = getEp()
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def getCO2_client():
	rospy.wait_for_service("setTemp")
	try:
		getCO2 = rospy.ServiceProxy("getCO2",HVAC_GetCO2)
		response = getCO2()
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def getRH_client():
	rospy.wait_for_service("setTemp")
	try:
		getRH = rospy.ServiceProxy("getRH",HVAC_GetRH)
		response = getRH()
		return response
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def usage():
	print("usage:")
	print("%s set_Temp [Temp]"%sys.argv[0])
	print("%s set_FanSp [FanSp]"%sys.argv[0])
	print("%s set_Ep [ep] [value]"%sys.argv[0])
	print("%s set_Bms"%sys.argv[0])
	print("%s get_Temp"%sys.argv[0])
	print("%s get_Ep"%sys.argv[0])
	print("%s get_CO2"%sys.argv[0])
	print("%s get_RH"%sys.argv[0])
	print("%s help"%sys.argv[0])

if __name__ == '__main__':
	arg1 = str(sys.argv[1]).lower()
	if (arg1 == "set_temp" and len(sys.argv) == 3):
		setTemp_client(float(sys.argv[2]))
	elif (arg1 == "set_fansp" and len(sys.argv) == 3):
		setFanSp_client(str(sys.argv[2]))
	elif (arg1 == "set_ep" and len(sys.argv) == 4):
		setEp_client(int(sys.argv[2]),int(sys.argv[3]))
	elif (arg1 == "set_bms" and len(sys.argv) == 2):
		setBms_client()
	elif (arg1 == "get_temp" and len(sys.argv) == 2):
		temp_data = getTemp_client()
		print(temp_data)
	elif (arg1 == "get_ep" and len(sys.argv) == 2):
		ep_data = getEp_client()
		print(ep_data)
	elif (arg1 == "get_co2" and len(sys.argv) == 2):
		co2_data = getCO2_client()
		print(co2_data)
	elif (arg1 == "get_rh" and len(sys.argv) == 2):
		rh_data = getRH_client()
		print(rh_data)
	elif (arg1 == "help" and len(sys.argv) == 2):
		print()
		print("set_Temp: sets the temperature of the system in degrees celsius")
		print("          first argument is a temperature in celsius as a float")
		print()
		print("set_FanSp: sets the fanspeed for the system")
		print("           first argument is the fan speed, string = off|low|medium|high")
		print()
		print("set_Ep: sets EP for the system")
		print("        first argument is ep value to set")
		print("        second argument is the value to set to, integer from 0-100")
		print()
		print("set_Bms: enables building maintenance system to take control")
		print()
		print("get_Temp: gets current temperature reading from each sensor")
		print()
		print("get_CO2: gets current CO2 reading in the room")
		print()
		print("get_RH: gets the current relative humidity in the room")
	else:
		usage()


