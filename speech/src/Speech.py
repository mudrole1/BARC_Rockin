#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import String

def main():
    rospy.init_node('speech')
    speech_sub = rospy.Subscriber("/speech", String, speech_cb)
    rospy.spin()

def speech_cb(msg):
    rospy.loginfo('Saying \"' + msg.data + '\"')
    os.system( 'espeak --stdout \"' + msg.data + '\" | paplay' )

if __name__ == '__main__':
    main()
