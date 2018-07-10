#!/usr/bin/env python

import sys
import os
import numpy
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from scr_control.srv import *
from scr_control.msg import *


def converter(distances, row_length):
	heatmap = numpy.asarray(distances)
	heatmap = numpy.reshape(heatmap, (len(distances)/row_length, row_length))

	return heatmap	


# Get a heatmap of the room (returns int16[] distances and int16 room_length)
def get_distances(sensor_id, debug=False):
	state = utils.service_call('get_distances', TOFGetDistances, [sensor_id])
	if debug:
		print(state)
	if state:
		return converter(state.data)

	return None

# Get a heatmap of the room (returns int16[] distances and int16 room_length)
def get_distances_all(debug=False):
	state = utils.service_call('get_distances_all', TOFGetDistancesAll, [])
	if debug:
		print(state)
	if state:
		return converter(state.data, 160)

	return None


# Start updating TOF sensors
def start_counting(debug=False):
	state = utils.service_call('set_counting', TOFSetCounting, [True])
	if debug:
		print("Starting to update TOF sensors...")
	if state:
		return state

# Start updating TOF sensors
def stop_counting(debug=False):
	state = utils.service_call('set_counting', TOFSetCounting, [False])
	if debug:
		print("Stopping TOF sensors...")
	if state:
		return state

# Show help regarding TOF commands
def help(debug=False):
	return utils.help(os.path.dirname(__file__), "SCR_TOF_help.txt", debug = debug)

if(__name__ == "__main__"):

					#command              #function        	   #argument types   #help
	serviceCalls = {
					'get_distances_all':  [get_distances_all,  [],               "get_distances_all"],
					'get_distances':      [get_distances,      [int],            "get_distances [sensor_id]"],
					'start_counting':     [start_counting,     [],               "start_counting"],
					'stop_counting':      [stop_counting,      [],               "stop_counting"],
					'help':               [help,               [],               "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True) 