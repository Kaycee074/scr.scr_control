#!/usr/bin/env python

#The light client allows the user to change the color of one light once
#or to change the color of one light between two colors a number of times
#The LightState and TestLightChange services  handle communication between
#the client and the server.

#To change light color once:
#rosrun light light_client.py address color

#To change between 2 colors x times:
#rosrun light light_client.py address color1 color2 x

#address is last 3 numbers of IP (111,112,...) in both cases

import sys
import rospy
import socket
from scr_control.srv import *
from scr_control.msg import *



def sample_light_client(address,state):
    rospy.wait_for_service('light_state')
    try:
        light_state = rospy.ServiceProxy('light_state',LightState)
        newState = light_state(address,state)
        return newState
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def test_light_change_client(adress,state1,state2,count):
    rospy.wait_for_service('test_light_change')
    try:
        test_light_change = rospy.ServiceProxy('test_light_change',TestLightChange)
        finalState = test_light_change(adress,state1,state2,count)
        return finalState
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)


def usage():
    return "%s light_name new_state\n%s light_name state1 state2 count" % (sys.argv[0],sys.argv[0])

if(__name__ == "__main__"):
    if (len(sys.argv) == 3):
        name = str(sys.argv[1])
        state = str(sys.argv[2])

        address = "192.168.0."+name

        print("Changing light %s to %s"%(address,state))

        new_state = sample_light_client(address,state)

        print("Light %s is now in state %s"%(address,new_state))

    elif (len(sys.argv) == 5):
        name = str(sys.argv[1])
        state1 = str(sys.argv[2])
        state2 = str(sys.argv[3])
        count = int(sys.argv[4])

        address = "192.168.0."+name

        print("Changing light %s between %s and %s %s times"%(address,state1,state2,count))

        final_state = test_light_change_client(address,state1,state2,count)

        print("Light %s changed between %s and %s %s times and is now in state %s"%(address,state1,state2,count,final_state))

    else:
        print usage()
        sys.exit(1)


    #pub = rospy.Publisher('sample_light_server', lightbulb, queue_size=10)

    #rospy.init_node('light_switch')

    #msg = lightbulb()
    #msg.name = host
    #msg.state = sample_light_client(name,state)
    #pub.publish(msg)
