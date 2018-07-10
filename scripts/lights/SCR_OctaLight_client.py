#!/usr/bin/env python

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), "../utils"))
import utils
from scr_control.srv import *
from scr_control.msg import *

# Change color of light using cct value and intensity
def cct(x, y, cct_val, intensity_val, debug=False):
	if debug:
		print("Changing light (%s,%s) to cct %s and intensity %s" % (x, y, cct_val, intensity_val))
	state = utils.service_call('cct', OctaLight_CCT, [cct_val, intensity_val, x, y])
	if state and debug:
		print("Light (%s,%s) is now in state %s" % (x, y, state))
	return state

# Change color of a light using primary sources
def sources(x, y, b1, b2, b3, l, a, o, r1, r2, debug=False):
	if debug:
		print("Changing light (%s,%s) to blue1:%s%% blue2:%s%% blue3:%s%% lime:%s%% amber:%s%% orange:%s%% red1:%s%% red2:%s%%" % (x, y, b1, b2, b3, l, a, o, r1, r2))
	state = utils.service_call('sources', OctaLight_sources, [b1, b2, b3, l, a, o, r1, r2, x, y])
	if state and debug:	
		print("Light (%s,%s) is now in state %s" % (x, y, state))
	return state

# Change color of a light using primary sources
def sources_all(b1, b2, b3, l, a, o, r1, r2, debug=False):
	if debug:
		print("Changing all lights to blue1:%s%% blue2:%s%% blue3:%s%% lime:%s%% amber:%s%% orange:%s%% red1:%s%% red2:%s%%" % (b1, b2, b3, l, a, o, r1, r2))
	state = utils.service_call('sources_all', OctaLight_sourcesAll, [b1, b2, b3, l, a, o, r1, r2])
	if state and debug:	
		print("Lights are now in state %s" % (state))
	return state

# Change color of light using cct value and intensity
def cct_all(cct_val, intensity_val, debug=False):
	if debug:
		print("Changing all lights to cct %s and intensity %s" % (cct_val, intensity_val))
	state = utils.service_call('cct_all', OctaLight_CCTAll, [cct_val, intensity_val])
	if state and debug:
		print("Lights are now in state %s" % (state))
	return state

# Get cct value of a given light (returns based on server memory)
def get_lights(debug=False):
	state = utils.service_call('get_lights', GetLights, [])
	if state:
		lights = []
		for i in range(0, len(state.lights), 2):
			lights.append((state.lights[i], state.lights[i+1]))
		if debug:
			print("The available lights are %s" % lights)
		return lights
	return None

# Get cct value of a given light (returns based on server memory)
def get_sources(x, y, debug=False):
	state = utils.service_call('get_sources', GetSources, [x, y])
	if state and debug:
		print("The sources of Light (%s,%s) is %s" % (x, y, state.sources))
	return state.sources

# Show help regarding light commands
def help(debug=False):
	return utils.help(os.path.dirname(__file__), "SCR_OctaLight_help.txt", debug = debug)

if(__name__ == "__main__"):

					#command     #function     #argument types                                         #help
	serviceCalls = {
					'cct':         [cct,         [int, int, int, int],                                 "cct [x_coord] [y_coord] [cct] [intensity]"],
					'sources':     [sources,     [int, int, int, int, int, int, int, int, int, int],   "sources [x_coord] [y_coord] [blue] [blue] [blue] [lime] [amber] [orange] [red] [red]"],
					'cct_all':     [cct_all,     [int, int],                                           "cct_all [cct] [intensity]"],
					'sources_all': [sources_all, [int, int, int, int, int, int, int, int],             "sources_all [blue] [blue] [blue] [lime] [amber] [orange] [red] [red]"],
					'get_sources': [get_sources, [int, int],                                           "get_sources [x_coord] [y_coord]"],
					'get_lights':  [get_lights,  [],                                                   "get_lights"],
					'help':        [help,        [],                                                   "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True)