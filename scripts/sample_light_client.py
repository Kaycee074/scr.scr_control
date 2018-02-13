#!/usr/bin/env python

import sys
import rospy
import socket
from light.srv import *
from light.msg import lightbulb

#from .. import Light

def sample_light_client(address,state):
    rospy.wait_for_service('light_state')
    try:
        light_state = rospy.ServiceProxy('light_state',LightState)
        newState = light_state(address,state)
        return newState
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def usage():
    return "%s light_name new_state" % sys.argv[0]

if(__name__ == "__main__"):
    if (len(sys.argv) == 3):
        name = str(sys.argv[1])
        state = str(sys.argv[2])
    else:
        print usage()
        sys.exit(1)

    address = "192.168.0."+name
    #port = 57007

    #blue = green = amber = red = white  = "0000"

    #if (state == "blue"):
    #    blue = "FFFF"
    #elif (state == "green"):
    #    green = "FFFF"
    #elif (state == "amber"):
    #    amber = "FFFF"
    #elif (state == "red"):
    #    red = "FFFF"
    #elif (state == "white"):
    #    white = "FFFF"

    #cmdstr = "PS"+blue+green+amber+red+white

    print("Changing light %s to %s"%(address,state))

    new_state = sample_light_client(address,state)

    print("Light %s is now in state %s"%(address,new_state))

    #pub = rospy.Publisher('sample_light_server', lightbulb, queue_size=10)

    #rospy.init_node('light_switch')

    #msg = lightbulb()
    #msg.name = host
    #msg.state = sample_light_client(name,state)
    #pub.publish(msg)
