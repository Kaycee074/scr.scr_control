#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from light.msg import lightbulb
#from .. import Light

def callback(data):
    rospy.loginfo("I heard light %s go to state %s"%(data.address,data.red))

def listener():
    rospy.init_node('sample_light1')
    rospy.Subscriber("sample_light_server", AddressColor, callback)
    #spin () keeps python from exiting until this node has stopped
    rospy.spin()

if __name__ == "__main__":
#    sample_light = Light(1,0)
    print('Listening')
    listener()
