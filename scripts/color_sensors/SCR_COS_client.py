#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from __future__ import print_function
from scr_control.srv import *
from scr_control.msg import *

# Get all values of color sensors
def readAll_client(debug=False):
	state = utils.service_call('readAll', COSReadAll, [])
	if state and debug:
		for i in range(len(state.data)):
			if(i%state.step == 0 and i > 1):
				print()
			print(state.data[i], end = " ")
	return state

# Read one color sensor
def readOne_client(num, debug=False):
	state = utils.service_call('readOne', COSReadOne, [num])
	if state and debug:
		print(state)
	return state

# Integration time
def inteTime_client(num, debug=False):
	state = utils.service_call('inteTime', COSInteTime, [num])
	if state and debug:
		print(state)
	return state

# Show help regarding color sensor commands
def help(debug=False):
	helpFile = open(os.path.join(os.path.dirname(__file__), 'SCR_COS_help.txt'))
	print(helpFile.read())
	helpFile.close()

if __name__ == '__main__':

					#command        #function           #argument types    #help
	serviceCalls = {
					'read_all':     [readAll_client,    [],                "read_all"],
					'read':         [readOne_client,    [int],             "read [sensor_num]"],
					'inte_time':    [inteTime_client,   [int],             "inte_time [time]"],
					'help':         [help,              [],                "help"]}
	
	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True)