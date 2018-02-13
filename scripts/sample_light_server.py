#!/usr/bin/env python

from light.srv import *
import rospy
import socket
#from .. import Light

def handle_light_state(req):
    port = 57007

    blue = green = amber = red = white  = "0000"

    if (req.color == "blue"):
        blue = "FFFF"
    elif (req.color == "green"):
        green = "FFFF"
    elif (req.color == "amber"):
        amber = "FFFF"
    elif (req.color == "red"):
        red = "FFFF"
    elif (req.color == "white"):
        white = "FFFF"

    cmdstr = "PS"+blue+green+amber+red+white

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((req.address,port))

    s.send(cmdstr)

    s.shutdown(1)
    s.close()

    return LightStateResponse(cmdstr)


    #print("Light %s is now %s"%(req.name,req.newState))
    #return(req.newState)

def light_state_server():
    rospy.init_node('light_state_server')
    # new service called light_state with LightState service type, requests passed to handle...
    s =  rospy.Service('light_state', LightState, handle_light_state)
    print("Ready to turn lights on")
    rospy.spin()

if(__name__ == "__main__"):
    light_state_server()
