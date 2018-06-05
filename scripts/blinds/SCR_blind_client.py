#!/usr/bin/env python

import sys
import os

sys.path.append('../utils')
from utils import *

# Lift or lower a given blind
def lift_client(b, val, debug=False):
	state = service_call('lift', BlindLift, [b, val])
	if state and debug:
		print("Blind %s is now lifted to %s%%" % (b, val))

# Tilt a given blind
def tilt_client(b, val, debug=False):
	state = service_call('tilt', BlindTilt, [b, val])
	if state and debug:
		print("Blind %s is now tilted to %s%%" % (b, val))

# Show help regarding blind commands
def help(debug=False):
	helpFile = open(os.path.join(os.path.dirname(__file__), 'SCR_blind_help.txt'))
	print(helpFile.read())
	helpFile.close()

if(__name__ == "__main__"):

					#command     #function        #argument types    #help
	serviceCalls = {'tilt':      [tilt_client,     [str, int],       "lift [blind] [percent]"],
					'lift':      [lift_client,     [str, int],       "tilt [blind] [percent]"],
					'help':      [help,            [],               "help"]}

	state = commandToFunction(sys.argv, serviceCalls, debug=True) 