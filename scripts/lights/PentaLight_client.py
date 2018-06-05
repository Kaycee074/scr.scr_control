#!/usr/bin/env python

import sys
import os

sys.path.append('../utils')
from utils import *

def CCT_client(x, y, CCT_val, intensity_val, debug=False):
	if debug:
		print("Changing light (%s,%s) to CCT %s and intensity %s" % (x, y, CCT_val, intensity_val))
	state = service_call('CCT', PentaLight_CCT, [CCT_val, intensity_val, x, y])
	if state and debug:
		print("Light (%s,%s) is now in state %s" % (x, y, state))

def ragbw_client(x, y, red_val, amber_val, green_val, blue_val, white_val, debug=False):
	if debug:
		print("Changing light (%s,%s) to red:%s%% amber:%s%% green:%s%% blue:%s%% white:%s%%" % (x, y, red_val, amber_val, green_val, blue_val, white_val))
	state = service_call('ragbw', PentaLight_ragbw, [red_val, amber_val, green_val, blue_val, white_val, x, y])
	if state and debug:	
		print("Light (%s,%s) is now in state %s" % (x, y, state))
	return state

def get_CCT_client(x, y, debug=False):
	state = service_call('getCCT', GetCCT, [x, y])
	if state and debug:
		print("The CCT of Light (%s,%s) is %s" % (x, y, state))
	return state

def get_int_client(x, y, debug=False):
	state = service_call('getInt', GetInt, [x, y])
	if state and debug:
		print("The intensity of Light (%s,%s) is %s" % (x, y, state))
	return state

def help(debug=False):
	helpFile = open(os.path.join(os.path.dirname(__file__), 'SCR_PentaLight_help.txt'))
	print(helpFile.read())
	helpFile.close()

if(__name__ == "__main__"):

					#command     #function        #argument types                      #help
	serviceCalls = {'cct': 	 	 [CCT_client,     [int, int, int, int],				   "CCT [x_coord] [y_coord] [CCT] [intensity]"],
					'ragbw': 	 [ragbw_client,   [int, int, int, int, int, int, int], "ragbw [x_coord] [y_coord] [red] [amber] [green] [blue] [white]]"],
					'get_cct':	 [get_CCT_client, [int, int],				           "get_CCT [x_coord] [y_coord]"],
					'get_int':	 [get_int_client, [int, int],				           "get_int [x_coord] [y_coord]"],
					'help':		 [help,           [],				  				   "help"]}

	state = commandToFunction(sys.argv, serviceCalls, debug=True)