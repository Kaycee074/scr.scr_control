#!/usr/bin/env python

from __future__ import print_function
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "../utils"))
import utils
from scr_control.srv import *
import numpy as np

# Get all values of color sensors
def read_all(debug=False):
	state = utils.service_call('read_all', COSReadAll, [])
	if state:
		out = []
		for i in range(len(state.data)):
			if(i%state.step == 0):
				out.append([])
			out[-1].append(state.data[i])
		if debug:
			for i in range(len(state.data)):
				if(i%state.step == 0 and i > 1):
					print()
				print(state.data[i], end = " ")
		return out

# Read one color sensor
def read(num, debug=False):
	state = utils.service_call('read', COSReadOne, [num])
	if state and debug:
		print(state)
	return state.data

# Integration time
def inte_time(num, debug=True):
	state = utils.service_call('inte_time', COSInteTime, [num])
	if state and debug:
		print(state)
	return state

# Show help regarding color sensor commands
def help(debug=False):
	return utils.help(os.path.dirname(__file__), "SCR_COS_help.txt", debug = debug)

if __name__ == '__main__':

					#command        #function     #argument types    #help
	serviceCalls = {
					'read_all':     [read_all,    [],                "read_all"],
					'read':         [read,        [int],             "read [sensor_num]"],
					'inte_time':    [inte_time,   [int],             "inte_time [time]"],
					'help':         [help,        [],                "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True)
