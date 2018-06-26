#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from scr_control.srv import *
from scr_control.msg import *

# Change color of light using cct value and intensity
def cct(x, y, cct_val, intensity_val, debug=False):
	if debug:
		print("Changing light (%s,%s) to cct %s and intensity %s" % (x, y, cct_val, intensity_val))
	state = utils.service_call('cct', PentaLight_cct, [cct_val, intensity_val, x, y])
	if state and debug:
		print("Light (%s,%s) is now in state %s" % (x, y, state))
	return state.cmdstr

# Change color of a light using Red/Amber/Green/Blue/White values
def ragbw(x, y, red_val, amber_val, green_val, blue_val, white_val, debug=False):
	if debug:
		print("Changing light (%s,%s) to red:%s%% amber:%s%% green:%s%% blue:%s%% white:%s%%" % (x, y, red_val, amber_val, green_val, blue_val, white_val))
	state = utils.service_call('ragbw', PentaLight_ragbw, [red_val, amber_val, green_val, blue_val, white_val, x, y])
	if state and debug:	
		print("Light (%s,%s) is now in state %s" % (x, y, state))
	return state.cmdstr

# Change color of a light using Red/Amber/Green/Blue/White values
def ragbw_all(red_val, amber_val, green_val, blue_val, white_val, debug=False):
	if debug:
		print("Changing all lights to red:%s%% amber:%s%% green:%s%% blue:%s%% white:%s%%" % (red_val, amber_val, green_val, blue_val, white_val))
	state = utils.service_call('ragbw_all', PentaLight_ragbwAll, [red_val, amber_val, green_val, blue_val, white_val])
	if state and debug:	
		print("Lights are now in state %s" % (state))
	return state.cmdstr

# Change color of light using cct value and intensity
def cct_all(cct_val, intensity_val, debug=False):
	if debug:
		print("Changing all lights to cct %s and intensity %s" % (cct_val, intensity_val))
	state = utils.service_call('cct_all', PentaLight_cctAll, [cct_val, intensity_val])
	if state and debug:
		print("Lights are now in state %s" % (state))
	return state.cmdstr

# Get cct value of a given light (returns based on server memory)
def get_lights(debug=False):
	state = utils.service_call('getLights', GetLights, [])
	if state:
		lights = []
		for i in range(0, len(state.lights), 2):
			lights.append((state.lights[i], state.lights[i+1]))
		if debug:
			print("The available lights are %s" % lights)
		return lights
	return None

# Get cct value of a given light (returns based on server memory)
def get_cct(x, y, debug=False):
	state = utils.service_call('getcct', Getcct, [x, y])
	if state and debug:
		print("The cct of Light (%s,%s) is %s" % (x, y, state))
	return state.cct

# Get INT value of a given light (returns based on server memory)
def get_int(x, y, debug=False):
	state = utils.service_call('getInt', GetInt, [x, y])
	if state and debug:
		print("The intensity of Light (%s,%s) is %s" % (x, y, state))
	return state.intensity

# Show help regarding light commands
def help(debug=False):
	helpFile = open(os.path.join(os.path.dirname(__file__), 'SCR_PentaLight_help.txt'), 'r')
	helpStr = helpFile.read()
	print(helpStr)
	helpFile.close()
	return helpStr
if(__name__ == "__main__"):

					#command     #function     #argument types                      #help
	serviceCalls = {
					'cct':       [cct,         [int, int, int, int],                "cct [x_coord] [y_coord] [cct] [intensity]"],
					'ragbw':     [ragbw,       [int, int, int, int, int, int, int], "ragbw [x_coord] [y_coord] [red] [amber] [green] [blue] [white]"],
					'cct_all':   [cct_all,     [int, int],                          "cct_all [cct] [intensity]"],
					'ragbw_all': [ragbw_all,   [int, int, int, int, int],           "ragbw_all [red] [amber] [green] [blue] [white]"],
					'get_cct':   [get_cct,     [int, int],                          "get_cct [x_coord] [y_coord]"],
					'get_int':   [get_int,     [int, int],                          "get_int [x_coord] [y_coord]"],
					'get_lights':[get_lights,  [],                                  "get_lights"],
					'help':      [help,        [],                                  "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True)