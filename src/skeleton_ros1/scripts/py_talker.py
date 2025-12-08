#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This script publishes messages to the 'chatter' topic at 10 Hz.
"""

import rospy
from std_msgs.msg import String

def talker():
    # Create a publisher object which allows us to publish messages to the 'chatter' topic
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # Initialize the node with the name 'py_talker'
    rospy.init_node('py_talker', anonymous=True)
    
    # Set the rate to 10 Hz
    rate = rospy.Rate(10) # 10hz
    
    # Counter for messages
    count = 0
    
    # Main loop
    while not rospy.is_shutdown():
        # Create the message
        hello_str = "Hello World from Python %d" % count
        
        # Log the message
        rospy.loginfo(hello_str)
        
        # Publish the message
        pub.publish(hello_str)
        
        # Sleep to maintain the publishing rate
        rate.sleep()
        
        # Increment counter
        count += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass