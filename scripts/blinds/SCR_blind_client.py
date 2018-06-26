#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from scr_control.srv import *
from scr_control.msg import *

# Lift or lower a given blind
def lift(b, val, debug=False):
	state = utils.service_call('lift', BlindLift, [b, val])
	if state and debug:
		print("Blind %s is now lifted to %s%%" % (b, state.lift))
	return state.lift

# Lift or lower all blinds
def lift_all(val, debug=False):
	state = utils.service_call('lift_all', BlindLiftAll, [val])
	if state and debug:
		print("All blinds now lifted to %s%%" % (state.lift))
	return state.lift

# Tilt a given blind
def tilt(b, val, debug=False):
	state = utils.service_call('tilt', BlindTilt, [b, val])
	if state and debug:
		print("Blind %s is now tilted to %s%%" % (b, state.tilt))
	return state.tilt

# Tilt all blinds
def tilt_all(val, debug=False):
	state = utils.service_call('tilt_all', BlindTiltAll, [val])
	if state and debug:
		print("All blinds now tilted to %s%%" % (state.tilt))
	return state.tilt

# Get a list of all blinds
def get_blinds(debug=False):
	state = utils.service_call('get_blinds', Get_blinds, [])
	if state:
		if debug:
			print("Blind names are %s" % state.blinds)
		return state.blinds
	return None

# Show help regarding blind commands
def help(debug=False):
	return utils.help(os.path.dirname(__file__), "SCR_blind_help.txt", debug = debug)

if(__name__ == "__main__"):

					#command     #function    #argument types   #help
	serviceCalls = {
					'tilt':      [tilt,       [str, int],       "tilt [blind] [percent]"],
					'lift':      [lift,       [str, int],       "lift [blind] [percent]"],
					'lift_all':  [lift_all,   [int],            "lift_all [percent]"],
					'tilt_all':  [tilt_all,   [int],            "tilt_all [percent]"],
					'get_blinds':[get_blinds, [],               "get_blinds"],
					'help':      [help,       [],               "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True) 