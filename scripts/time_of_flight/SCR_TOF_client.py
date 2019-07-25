#!/usr/bin/env python

import sys
import os
import numpy
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from scr_control.srv import *


# Get a heatmap of the room (returns int16[] distances and int16 room_length)
def get_distances_all(debug=False):
	state = utils.service_call('get_distances_all', TOFGetDistancesAll, [])
	if state:
		dist = numpy.asarray(state.data)
		dist = numpy.reshape(dist, (len(state.data)/160, 160))
		dist = dist.tolist()
		if debug:
			print(dist)
		return dist

# Get a heatmap of the room (returns int16[] distances and int16 room_length)
def get_distances(debug=False):
	state = utils.service_call('get_distances', TOFGetDistances, [])
	if state:
		dist = numpy.asarray(state.data)
		dist = numpy.reshape(dist, (len(state.data)/20, 20))
		dist = dist.tolist()
		if debug:
			print(dist)
		return dist

# Show help regarding TOF commands
def help(debug=False):
	return utils.help(os.path.dirname(__file__), "SCR_TOF_help.txt", debug = debug)

if(__name__ == "__main__"):
	print (os.path.join(os.path.dirname(sys.path[0]), "utils"))
					#command         	 #function      	 #argument types   #help
	serviceCalls = {
					'get_distances_all': [get_distances_all, [],               "get_distances_all"],
					'get_distances': 	 [get_distances,	 [],               "get_distances"],
					'help':          	 [help,         	 [],               "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True)
