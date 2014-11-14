#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import String
from speech.srv import *

def main():
    rospy.init_node('speech')
    rospy.Service('speech', Speech, speech_cb)
    rospy.spin()

def speech_cb(request):
    rospy.loginfo('Saying \"' + request.message.data + '\"')
    os.system( 'espeak --stdout \"' + request.message.data + '\" | paplay' )
    return SpeechResponse()

if __name__ == '__main__':
    main()
