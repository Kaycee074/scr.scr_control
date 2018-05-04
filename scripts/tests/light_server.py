#!/usr/bin/env python

#The light server recieves address, color(s), and an optional
#count from the client.
#Using the color(s), command strings are created.
#Using the address, a socket for a specific light is opended.
#The command string is sent to the light.
#If a second color and a count were recieved, the light will
#change between the two colors 'count' times at a rate
#specified by the delay variable.

#To run server:
#rosrun light light_server.py

#In order to change the rate color is changed at, the server
#must be stopped using ^C and catkin_make must be called in
#~/catkin_ws before running the server again

#The LightState and the TestLightChange services are used to
#communicate between the server and the client.

from scr_control.srv import *
import rospy
import socket
import time

def gen_cmdstr(state):
    """enerates command string based on color recieved by client"""
    blue = green = amber = red = white  = "0000"

    if (state == "blue"):
        blue = "FFFF"
    elif (state == "green"):
        green = "FFFF"
    elif (state == "amber"):
        amber = "FFFF"
    elif (state == "red"):
        red = "FFFF"
    elif (state == "white"):
        white = "FFFF"

    return "PS"+blue+green+amber+red+white

def handle_light_state(req):
    port = 57007

    cmdstr = gen_cmdstr(req.color)

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((req.address,port))
    except ConnectionRefusedError:
        rs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rs.connect((req.address,57011))
	print("Connection refused on " + req.address)

    s.send(cmdstr)
    a = s.recv(1024)
    time.sleep(1)

    s.shutdown(socket.SHUT_RDWR)
    s.close()

    return LightStateResponse(cmdstr)

def handle_test_light_change(req):
    port = 57007
    cmdstr1 = gen_cmdstr(req.color1)
    cmdstr2 = gen_cmdstr(req.color2)

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((req.address,port))
    except ConnectionRefusedError:
        rs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rs.connect((req.address,57011))
	print("Connection refused on " + req.address)

    delay = 0.005

    #sends the two command strings count times at specified rate
    for i in range(req.count):
        s.send(cmdstr1)
        a = s.recv(1024)
        time.sleep(delay)

        s.send(cmdstr2)
        a = s.recv(1024)
        time.sleep(delay)

    s.shutdown(socket.SHUT_RDWR)
    s.close()

    return TestLightChangeResponse(cmdstr2)

def light_state_server():
    rospy.init_node('light_state_server')
    # new service called light_state with LightState service type, requests passed to handle...
    state_serv =  rospy.Service('light_state', LightState, handle_light_state)
    # new service called test_light_change with TestLightChange service type, requests passed to handle...
    change_serv = rospy.Service('test_light_change', TestLightChange, handle_test_light_change)
    print("Ready to turn lights on")
    rospy.spin()

if(__name__ == "__main__"):
    light_state_server()
