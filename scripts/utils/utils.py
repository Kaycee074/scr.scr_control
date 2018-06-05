import rospy
import socket
from scr_control.srv import *
from scr_control.msg import *

# Makes a service call to the server
def service_call(serviceName, service, arguments):
	rospy.wait_for_service(serviceName)
	try:
		function = rospy.ServiceProxy(serviceName, service)
		state = function(*arguments)
		return state
	except rospy.ServiceException, e:
		print("Service call failed: %s" % e)
		return None

# Runs an API function using sys arguments from command line. See any client file for format of validCommands
def commandToFunction(sysargs, validCommands, debug=False):
	command = str(sysargs[1]).lower()
	state = None
	if command in validCommands:
		function, argumentTypes, helpStr = validCommands[command][0], validCommands[command][1], validCommands[command][2]
		if len(sysargs) == len(argumentTypes) + 2:
			arguments = []
			for i in range(len(argumentTypes)):
				try:
					arguments.append(argumentTypes[i](sysargs[i+2]))
				except:
					print("Invalid Arguments! Use: " + helpStr)
					break
			if len(arguments) == len(argumentTypes):
				state = function(*arguments, debug=debug)

		else:
			print("Invalid Arguments! Use: " + helpStr)
	else:
		print("Valid Commands:")
		for c in validCommands:
			print(validCommands[c][2])
	return state