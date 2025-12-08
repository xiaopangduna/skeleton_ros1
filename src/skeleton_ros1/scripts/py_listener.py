#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This script subscribes to the 'chatter' topic and prints received messages.
"""

import rospy
from std_msgs.msg import String

def callback(data):
    """Callback function that gets called when a message is received."""
    rospy.loginfo("I heard from Python: %s", data.data)

def listener():
    # Initialize the node with the name 'py_listener'
    rospy.init_node('py_listener', anonymous=True)
    
    # Subscribe to the 'chatter' topic with the callback function
    rospy.Subscriber('chatter', String, callback)
    
    # Keep the node alive until it's shut down
    rospy.spin()

if __name__ == '__main__':
    listener()