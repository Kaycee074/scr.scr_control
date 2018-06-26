#!/usr/bin/env python

from __future__ import print_function
import sys, os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from scr_control.srv import *
from scr_control.msg import *

def set_temp(temp, debug = False):
	state = utils.service_call('set_temp', HVAC_SetTemp, [temp])
	if state and debug:
		print(state)

def set_fansp(speed, debug = False):
	state = utils.service_call('set_fansp', HVAC_SetFanSp, [speed])
	if state and debug:
		print(state)

def set_ep(ep, val, debug = False):
	state = utils.service_call('set_ep', HVAC_SetEp, [ep, val])
	if state and debug:
		print(state)

def set_bms(debug = False):
	state = utils.service_call('set_bms', HVAC_SetBms, [])
	if state and debug:
		print(state)

def get_temp(debug = False):
	state = utils.service_call('get_temp', HVAC_GetTemp, [])
	if state:
		if debug:
			print(state)
		return state.data

def get_ep(debug = False):
	state = utils.service_call('get_ep', HVAC_GetEp, [])
	if state:
		if debug:
			print(state)
		return state.data

def get_co2(debug = False):
	state = utils.service_call('get_co2', HVAC_GetCO2, [])
	if state:
		if debug:
			print(state)
		return state.data

def get_rh(debug = False):
	state = utils.service_call('get_rh', HVAC_GetRH, [])
	if state:
		if debug:
			print(state)
		return state.data
		
# Show help regarding blind commands
def help(debug=False):
	helpFile = open(os.path.join(os.path.dirname(__file__), 'SCR_HVAC_help.txt'))
	helpStr = helpFile.read()
	print(helpStr)
	helpFile.close()
	return helpStr

if(__name__ == "__main__"):

					#command     #function    #argument types   #help
	serviceCalls = {
					'set_temp':  [set_temp,   [float],          "set_temp [temp]"],
					'set_fansp': [set_fansp,  [str],            "set_fansp [speed = off|low|medium|high]"],
					'set_ep':    [set_ep,     [int, int],       "set_ep [ep] [value]"],
					'set_bms':   [set_bms,    [],               "set_bms"],
					'get_temp':  [get_temp,   [],               "get_temp"],
					'get_ep':    [get_ep,     [],               "get_ep"],
					'get_co2':   [get_co2,    [],               "get_co2"],
					'get_rh':    [get_rh,     [],               "get_rh"],
					'help':      [help,       [],               "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True) 