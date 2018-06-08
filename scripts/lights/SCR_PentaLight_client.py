#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from scr_control.srv import *
from scr_control.msg import *

# Change color of light using CCT value and intensity
def CCT_client(x, y, CCT_val, intensity_val, debug=False):
	if debug:
		print("Changing light (%s,%s) to CCT %s and intensity %s" % (x, y, CCT_val, intensity_val))
	state = utils.service_call('CCT', PentaLight_CCT, [CCT_val, intensity_val, x, y])
	if state and debug:
		print("Light (%s,%s) is now in state %s" % (x, y, state))
	return state

# Change color of a light using Red/Amber/Green/Blue/White values
def ragbw_client(x, y, red_val, amber_val, green_val, blue_val, white_val, debug=False):
	if debug:
		print("Changing light (%s,%s) to red:%s%% amber:%s%% green:%s%% blue:%s%% white:%s%%" % (x, y, red_val, amber_val, green_val, blue_val, white_val))
	state = utils.service_call('ragbw', PentaLight_ragbw, [red_val, amber_val, green_val, blue_val, white_val, x, y])
	if state and debug:	
		print("Light (%s,%s) is now in state %s" % (x, y, state))
	return state

# Change color of a light using Red/Amber/Green/Blue/White values
def ragbw_all_client(red_val, amber_val, green_val, blue_val, white_val, debug=False):
	if debug:
		print("Changing all lights to red:%s%% amber:%s%% green:%s%% blue:%s%% white:%s%%" % (red_val, amber_val, green_val, blue_val, white_val))
	state = utils.service_call('ragbw_all', PentaLight_ragbwAll, [red_val, amber_val, green_val, blue_val, white_val])
	if state and debug:	
		print("Lights are now in state %s" % (state))
	return state

# Change color of light using CCT value and intensity
def CCT_all_client(CCT_val, intensity_val, debug=False):
	if debug:
		print("Changing all lights to CCT %s and intensity %s" % (CCT_val, intensity_val))
	state = utils.service_call('CCT_all', PentaLight_CCTAll, [CCT_val, intensity_val])
	if state and debug:
		print("Lights are now in state %s" % (state))
	return state

# Get CCT value of a given light (returns based on server memory)
def get_lights_client(debug=False):
	state = utils.service_call('getLights', GetLights, [])
	if state:
		lights = []
		for i in range(0, len(state.lights), 2):
			lights.append((state.lights[i], state.lights[i+1]))
		if debug:
			print("The avialable lights are %s" % lights)
		return lights
	return state

# Get CCT value of a given light (returns based on server memory)
def get_CCT_client(x, y, debug=False):
	state = utils.service_call('getCCT', GetCCT, [x, y])
	if state and debug:
		print("The CCT of Light (%s,%s) is %s" % (x, y, state))
	return state

# Get INT value of a given light (returns based on server memory)
def get_int_client(x, y, debug=False):
	state = utils.service_call('getInt', GetInt, [x, y])
	if state and debug:
		print("The intensity of Light (%s,%s) is %s" % (x, y, state))
	return state

# Show help regarding light commands
def help(debug=False):
	helpFile = open(os.path.join(os.path.dirname(__file__), 'SCR_PentaLight_help.txt'), 'r')
	print(helpFile.read())
	helpFile.close()

if(__name__ == "__main__"):

					#command     #function            #argument types                      #help
	serviceCalls = {
					'cct':       [CCT_client,         [int, int, int, int],                "CCT [x_coord] [y_coord] [CCT] [intensity]"],
					'ragbw':     [ragbw_client,       [int, int, int, int, int, int, int], "ragbw [x_coord] [y_coord] [red] [amber] [green] [blue] [white]"],
					'cct_all':   [CCT_all_client,     [int, int],                          "cct_all [CCT] [intensity]"],
					'ragbw_all': [ragbw_all_client,   [int, int, int, int, int],           "ragbw_all [red] [amber] [green] [blue] [white]"],
					'get_cct':   [get_CCT_client,     [int, int],                          "get_CCT [x_coord] [y_coord]"],
					'get_int':   [get_int_client,     [int, int],                          "get_int [x_coord] [y_coord]"],
					'get_lights':[get_lights_client,  [],                                  "get_lights"],
					'help':      [help,               [],                                  "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True)