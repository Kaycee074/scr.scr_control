#!/usr/bin/env python

from __future__ import print_function
import sys
import os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from scr_control.srv import *
from scr_control.msg import *

# Get all values of color sensors
def read_all(debug=False):
	state = utils.service_call('read_all', COSRead_all, [])
	if state and debug:
		for i in range(len(state.data)):
			if(i%state.step == 0 and i > 1):
				print()
			print(state.data[i], end = " ")
	return state.data

# Read one color sensor
def read(num, debug=False):
	state = utils.service_call('read', COSRead_one, [num])
	if state and debug:
		print(state)
	return state.data

# Integration time
def inte_time(num, debug=False):
	state = utils.service_call('inte_time', COSInte_time, [num])
	if state and debug:
		print(state)
	return state

# Show help regarding color sensor commands
def help(debug=False):
	helpFile = open(os.path.join(os.path.dirname(__file__), 'SCR_COS_help.txt'))
	print(helpFile.read())
	helpFile.close()

if __name__ == '__main__':

					#command        #function     #argument types    #help
	serviceCalls = {
					'read_all':     [read_all,    [],                "read_all"],
					'read':         [read,        [int],             "read [sensor_num]"],
					'inte_time':    [inte_time,   [int],             "inte_time [time]"],
					'help':         [help,        [],                "help"]}
	
	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True)