#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from scr_control.srv import *
from scr_control.msg import *

# Get a heatmap of the room (returns int16[] distances and int16 room_length)
def get_heat_map(debug=False):
	state = utils.service_call('get_heatmap', GetHeatmap, [])
	if state and debug:
		print(state.distances)
	if state:
		return state

# Show help regarding TOF commands
def help(debug=False):
	helpFile = open(os.path.join(os.path.dirname(__file__), 'SCR_TOF_help.txt'))
	print(helpFile.read())
	helpFile.close()

if(__name__ == "__main__"):

					#command       #function      #argument types    #help
	serviceCalls = {
					'get_heatmap': [get_heat_map, [],               "get_heatmap"],
					'help':        [help,         [],               "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True) 