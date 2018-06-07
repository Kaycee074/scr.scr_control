#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "utils"))
import utils
from scr_control.srv import *
from scr_control.msg import *

# Lift or lower a given blind
def lift_client(b, val, debug=False):
	state = utils.service_call('lift', BlindLift, [b, val])
	if state and debug:
		print("Blind %s is now lifted to %s%%" % (b, state.lift))

# Lift or lower all blinds
def liftAll_client(val, debug=False):
	state = utils.service_call('lift_all', BlindLiftAll, [val])
	if state and debug:
		print("All blinds now lifted to %s%%" % (state.lift))

# Tilt a given blind
def tilt_client(b, val, debug=False):
	state = utils.service_call('tilt', BlindTilt, [b, val])
	if state and debug:
		print("Blind %s is now tilted to %s%%" % (b, state.tilt))

# Tilt all blinds
def tiltAll_client(val, debug=False):
	state = utils.service_call('tilt_all', BlindTiltAll, [val])
	if state and debug:
		print("All blinds now tilted to %s%%" % (state.tilt))

# Get a list of all blinds
def getBlinds_client(debug=False):
	state = utils.service_call('get_blinds', GetBlinds, [])
	if state and debug:
		print("Blind names are %s" % state.blinds)
	return state

# Show help regarding blind commands
def help(debug=False):
	helpFile = open(os.path.join(os.path.dirname(__file__), 'SCR_blind_help.txt'))
	print(helpFile.read())
	helpFile.close()

if(__name__ == "__main__"):

					#command     #function          #argument types    #help
	serviceCalls = {
					'tilt':      [tilt_client,      [str, int],       "tilt [blind] [percent]"],
					'lift':      [lift_client,      [str, int],       "lift [blind] [percent]"],
					'lift_all':  [liftAll_client,   [int],            "lift_all [percent]"],
					'tilt_all':  [tiltAll_client,   [int],            "tilt_all [percent]"],
					'get_blinds':[getBlinds_client, [],               "get_blinds"],
					'help':      [help,             [],               "help"]}

	state = utils.commandToFunction(sys.argv, serviceCalls, debug=True) 