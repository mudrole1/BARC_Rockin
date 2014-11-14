#!/usr/bin/env python

import rospy
import smach
import smach_ros
import os
from std_msgs.msg import Empty, Int32, Bool, String, Float64MultiArray
from speech.srv import *

speech_pub = rospy.Publisher("/speech", String)

DEBUG = 1

FACE_CAMERA_MESSAGE = 'Please look at the camera'
UNKNOWN_GO_AWAY_MESSAGE = 'Sorry, I don\'t know you. I cannot open the door'
POSTMAN_HELLO_MESSAGE = 'Hello postman. Please come in'
POSTMAN_LEAVE_POST_MESSAGE = 'Please leave the post on the table'
DELIMAN_HELLO_MESSAGE = 'Hello deli man. Please come in'
DELIMAN_BREAKFAST_MESSAGE = 'Please put the breakfast on the table'
DOCTOR_HELLO_MESSAGE = 'Hello doctor. Please come in. I will take you to the bedroom'
DOCTOR_WAIT_MESSAGE = 'I will wait here'
FOLLOW_MESSAGE = 'Please follow me'
BYE_MESSAGE = 'Thank you. Good bye'
UNRECOGNISED_PERSON_MESSAGE = 'Hello visitor. Who are you and what is the purpose of your visit?'
ARE_YOU_MESSAGE = 'Are you '
YES_NO_MESSAGE = 'Please answer with yes or no.'
ARE_YOU_EXPECTED_MESSAGE = 'Is Granny Annie expecting you?'

class Listen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Bell'])
        self.result_sub = rospy.Subscriber("/bell_listener/response", Empty, self.result_cb)

    def execute(self, userdata):
        log('In state LISTEN')

        self.executed = False

        if DEBUG:
            self.executed = True

        while not rospy.is_shutdown():
            if self.executed:
                Navigation.destination = 'Door'
                Navigation.next_state = 'RECOGNITION'	
                return 'Bell'
            else:
                rospy.sleep(0.2)

    def result_cb(self, msg):
        self.executed = True

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SPEECH','RECOGNITION','DELIMAN_IN_KITCHEN','DOCTOR_IN_BEDROOM','BYE','Failed','Unknown'])
        self.run_pub = rospy.Publisher("/navigation/request", String)
        self.result_sub = rospy.Subscriber("/navigation/response", Int32, self.result_cb)

    def execute(self, userdata):
        log('In state NAVIGATION. Destination: ' + Navigation.destination)

        self.executed = False

        if DEBUG:
            self.result = 3
            self.executed = True

        self.run_pub.publish(Navigation.destination)

        while not rospy.is_shutdown():
            if self.executed:
                if self.result == 3:
                    return Navigation.next_state
                elif self.result == 2 or self.result == 4:
                    return 'Failed'
                elif self.result == -1:
                    rospy.logerr('Unknown Destination: ' + Navigation.destination)
                    return 'Unknown'
            else:
                rospy.sleep(0.2)

    def result_cb(self, msg):
        self.result = msg.data
        log(self.result)
        self.executed = True
		
class Recognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Doctor','Deliman','Postman','Unknown','No Face','Unrecognised','Unsure'])
        self.run_pub = rospy.Publisher("/recognition/request", Empty)
        self.stop_pub = rospy.Publisher("/recognition/stop", Empty)
        self.result_sub = rospy.Subscriber("/recognition/response", String, self.result_cb)
        self.speech_result_sub = rospy.Subscriber("/speech_rec/response", String, self.speech_result_cb)

    def execute(self, userdata):
        log('In state RECOGNITION')

        self.executed = False
        self.speech_received = False
        self.last_speech = rospy.get_rostime()
		
        self.run_pub.publish()

        while not rospy.is_shutdown():
            if self.executed:
                if self.person in ('Doctor'): # Camera successfully recognised the person
                    Recognition.unknown_count = 0
                    self.stop_pub.publish()
                    return self.person
                elif self.person in ('Deliman','Postman'): # Camera is unsure about the recognised person
                    ConfirmRecognition.person = self.person
                    self.stop_pub.publish()
                    self.confirm_speech()
                    return 'Unsure'
                elif self.person == 'No Face': # Person is not facing the camera
                    SpeechState.msg = FACE_CAMERA_MESSAGE
                    SpeechState.next_state = 'RECOGNITION'
                    return self.person
                elif Recognition.unknown_count > 2: # Person has not been recognised after > 2 recognition loops, assume unknown person
                    self.stop_pub.publish()
                    return 'Unknown'
                elif self.speech_received and rospy.get_rostime() - self.last_speech < rospy.Duration(5.0) and self.speech_person in ('Doctor','Deliman','Postman','Unknown'): # Camera doesn't recognise the person but speech has a response
                    Recognition.unknown_count = 0
                    ConfirmRecognition.person = self.speech_person
                    self.stop_pub.publish()
                    self.confirm_speech()
                    return 'Unsure'
                else: # Person not recognised, try another loop
                    Recognition.unknown_count += 1
                    SpeechState.msg = UNRECOGNISED_PERSON_MESSAGE
                    SpeechState.next_state = 'RECOGNITION'
                    return 'Unrecognised'
            else:
                rospy.sleep(0.2)

    def result_cb(self, msg):
        self.person = msg.data
        self.executed = True

    def speech_result_cb(self, msg):
        self.speech_person = msg.data
        self.last_speech = rospy.get_rostime()
        self.speech_received = True

    def confirm_speech(self):
        if ConfirmRecognition.person == 'Unknown':
            SpeechState.msg = (ARE_YOU_EXPECTED_MESSAGE + ' ' + YES_NO_MESSAGE)
        elif ConfirmRecognition.person == 'Doctor':
            SpeechState.msg = (ARE_YOU_MESSAGE + 'Doctor Kimble? ' + YES_NO_MESSAGE)
        elif ConfirmRecognition.person == 'Deliman':
            SpeechState.msg = (ARE_YOU_MESSAGE + 'the deli man? ' + YES_NO_MESSAGE)
        elif ConfirmRecognition.person == 'Postman':
            SpeechState.msg = (ARE_YOU_MESSAGE + 'the post man? ' + YES_NO_MESSAGE)
        else:
            rospy.error('Unknown person: ' + ConfirmRecognition.person)

        SpeechState.next_state = 'CONFIRM_RECOGNITION'

class ConfirmRecognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Doctor','Postman','Deliman','Unknown','Unsure','Unrecognised'])
        self.run_pub = rospy.Publisher("/speech_rec/confirmation/request", Empty)
        self.result_sub = rospy.Subscriber("/speech_rec/confirmation/response", Int32, self.result_cb)

    def execute(self, userdata):
        log('In state CONFIRM_RECOGNITION')

        self.executed = False

        while not rospy.is_shutdown():
            if self.executed:
                if self.result == -1: # Speech confirmation node was unsure what was said
                    return 'Unsure'
                elif ConfirmRecognition.person == 'Unknown':
                    if self.result == 1:
                        return 'Unrecognised' # Person is not unknown person, return to recognition
                    else:
                        return 'Unknown' # Person is unknown person
                else:
                    if self.result == 1:
                        return ConfirmRecognition.person # Person is as believed
                    else:
                        return 'Unrecognised' # Person is not as believed, return to recognition

    def result_cb(self, msg):
        self.result = msg.data
        self.executed = True

class SpeechState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['NAVIGATION','RECOGNITION','LISTEN','POSTMAN_ENTERED','POSTMAN_WAIT','DELIMAN_ENTERED','DELIMAN_WAIT','DOCTOR_ENTERED','DOCTOR_WAIT','CONFIRM_RECOGNITION'])

    def execute(self, userdata):
        log('In state SPEECH. Message: ' + SpeechState.msg)

        say(SpeechState.msg)
        rospy.sleep(1)
		
        return SpeechState.next_state

class Doctor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['NAVIGATION'])

    def execute(self, userdata):
        log('Doctor detected')

        Navigation.destination = 'Enter'
        Navigation.next_state = 'SPEECH'
        SpeechState.msg = DOCTOR_HELLO_MESSAGE
        SpeechState.next_state = 'DOCTOR_ENTERED'
		
        return 'NAVIGATION'

class Deliman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['NAVIGATION'])

    def execute(self, userdata):
        log('Deliman detected')

        Navigation.destination = 'Enter'
        Navigation.next_state = 'SPEECH'
        SpeechState.msg = DELIMAN_HELLO_MESSAGE
        SpeechState.next_state = 'DELIMAN_ENTERED'
		
        return 'NAVIGATION'

class Postman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['NAVIGATION'])

    def execute(self, userdata):
        log('Postman detected')

        Navigation.destination = 'Enter'
        Navigation.next_state = 'SPEECH'
        SpeechState.msg = POSTMAN_HELLO_MESSAGE
        SpeechState.next_state = 'POSTMAN_ENTERED'
		
        return 'NAVIGATION'

class UnknownPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):
        log('Unknown person detected')

        SpeechState.msg = UNKNOWN_GO_AWAY_MESSAGE
        SpeechState.next_state = 'LISTEN'
		
        return 'Speech'

class PostmanEntered(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):

        SpeechState.msg = POSTMAN_LEAVE_POST_MESSAGE
        SpeechState.next_state = 'POSTMAN_WAIT'
		
        return 'Speech'

class PostmanWait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Leaving'])
        self.run_pub = rospy.Publisher("/wait_for_postman/request", Empty)
        self.result_sub = rospy.Subscriber("/wait_for_postman/response", Empty, self.result_cb)

    def execute(self, userdata):
        self.executed = False

        self.run_pub.publish()

        if DEBUG:
            self.executed = True

        while not rospy.is_shutdown():
            if self.executed:
                return 'Leaving'
            else:
                rospy.sleep(0.2)

    def result_cb(self, msg):
        self.executed = True

class DelimanEntered(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):

        SpeechState.msg = FOLLOW_MESSAGE
        SpeechState.next_state = 'NAVIGATION'
        Navigation.destination = 'Kitchen'
        Navigation.next_state = 'DELIMAN_IN_KITCHEN'
		
        return 'Speech'

class DelimanInKitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):

        SpeechState.msg = DELIMAN_BREAKFAST_MESSAGE
        SpeechState.next_state = 'DELIMAN_WAIT'
		
        return 'Speech'

class DelimanWait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Leaving'])
        self.run_pub = rospy.Publisher("/wait_for_deliman/request", Empty)
        self.result_sub = rospy.Subscriber("/wait_for_deliman/response", Empty, self.result_cb)

    def execute(self, userdata):
        self.executed = False

        self.run_pub.publish()

        if DEBUG:
            self.executed = True

        while not rospy.is_shutdown():
            if self.executed:
                SpeechState.msg = FOLLOW_MESSAGE
                SpeechState.next_state = 'NAVIGATION'
                Navigation.destination = 'Enter'
                Navigation.next_state = 'BYE'
                return 'Leaving'
            else:
                rospy.sleep(0.2)

    def result_cb(self, msg):
        self.executed = True

class DoctorEntered(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):

        SpeechState.msg = FOLLOW_MESSAGE
        SpeechState.next_state = 'NAVIGATION'
        Navigation.destination = 'Bedroom'
        Navigation.next_state = 'DOCTOR_IN_BEDROOM'
		
        return 'Speech'

class DoctorInBedroom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):

        SpeechState.msg = DOCTOR_WAIT_MESSAGE
        SpeechState.next_state = 'DOCTOR_WAIT'
		
        return 'Speech'

class DoctorWait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Leaving'])
        self.run_pub = rospy.Publisher("/wait_for_doctor/request", Empty)
        self.result_sub = rospy.Subscriber("/wait_for_doctor/response", Empty, self.result_cb)

    def execute(self, userdata):
        self.executed = False

        self.run_pub.publish()

        if DEBUG:
            self.executed = True

        while not rospy.is_shutdown():
            if self.executed:
                SpeechState.msg = FOLLOW_MESSAGE
                SpeechState.next_state = 'NAVIGATION'
                Navigation.destination = 'Enter'
                Navigation.next_state = 'BYE'
                return 'Leaving'
            else:
                rospy.sleep(0.2)

    def result_cb(self, msg):
        self.executed = True

class Bye(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):
        log('In state BYE')

        SpeechState.msg = BYE_MESSAGE
        SpeechState.next_state = 'LISTEN'
		
        return 'Speech'
	
def main():
    rospy.init_node('state_machine')

    sm = smach.StateMachine(outcomes=['END'])

    with sm:
        smach.StateMachine.add('LISTEN', Listen(), transitions={'Bell':'NAVIGATION'})
        smach.StateMachine.add('NAVIGATION', Navigation(), transitions={'SPEECH':'SPEECH','RECOGNITION':'RECOGNITION','DELIMAN_IN_KITCHEN':'DELIMAN_IN_KITCHEN','DOCTOR_IN_BEDROOM':'DOCTOR_IN_BEDROOM','BYE':'BYE','Failed':'NAVIGATION','Unknown':'END'})
        smach.StateMachine.add('RECOGNITION', Recognition(), transitions={'Doctor':'DOCTOR','Deliman':'DELIMAN','Postman':'POSTMAN','Unknown':'UNKNOWN_PERSON','Unrecognised':'SPEECH','No Face':'SPEECH','Unsure':'SPEECH'})
        smach.StateMachine.add('CONFIRM_RECOGNITION', ConfirmRecognition(), transitions={'Doctor':'DOCTOR','Deliman':'DELIMAN','Postman':'POSTMAN','Unknown':'UNKNOWN_PERSON','Unrecognised':'RECOGNITION','Unsure':'CONFIRM_RECOGNITION'})
        smach.StateMachine.add('SPEECH', SpeechState(), transitions={'NAVIGATION':'NAVIGATION','RECOGNITION':'RECOGNITION','LISTEN':'LISTEN','POSTMAN_ENTERED':'POSTMAN_ENTERED','POSTMAN_WAIT':'POSTMAN_WAIT','DELIMAN_ENTERED':'DELIMAN_ENTERED','DELIMAN_WAIT':'DELIMAN_WAIT','DOCTOR_ENTERED':'DOCTOR_ENTERED','DOCTOR_WAIT':'DOCTOR_WAIT','CONFIRM_RECOGNITION':'CONFIRM_RECOGNITION'})
        smach.StateMachine.add('DOCTOR', Doctor(), transitions={'NAVIGATION':'NAVIGATION'})
        smach.StateMachine.add('DELIMAN', Deliman(), transitions={'NAVIGATION':'NAVIGATION'})
        smach.StateMachine.add('POSTMAN', Postman(), transitions={'NAVIGATION':'NAVIGATION'})
        smach.StateMachine.add('UNKNOWN_PERSON', UnknownPerson(), transitions={'Speech':'SPEECH'})
        smach.StateMachine.add('POSTMAN_ENTERED', PostmanEntered(), transitions={'Speech':'SPEECH'})
        smach.StateMachine.add('POSTMAN_WAIT', PostmanWait(), transitions={'Leaving':'BYE'})
        smach.StateMachine.add('DELIMAN_ENTERED', DelimanEntered(), transitions={'Speech':'SPEECH'})
        smach.StateMachine.add('DELIMAN_IN_KITCHEN', DelimanInKitchen(), transitions={'Speech':'SPEECH'})
        smach.StateMachine.add('DELIMAN_WAIT', DelimanWait(), transitions={'Leaving':'SPEECH'})
        smach.StateMachine.add('DOCTOR_ENTERED', DoctorEntered(), transitions={'Speech':'SPEECH'})
        smach.StateMachine.add('DOCTOR_IN_BEDROOM', DoctorInBedroom(), transitions={'Speech':'SPEECH'})
        smach.StateMachine.add('DOCTOR_WAIT', DoctorWait(), transitions={'Leaving':'SPEECH'})
        smach.StateMachine.add('BYE', Bye(), transitions={'Speech':'SPEECH'})

    rospy.sleep(2)

    Recognition.unknown_count = 0

    outcome = sm.execute()

def log(msg):
    rospy.loginfo(msg)
	
def say(msg):
    rospy.wait_for_service('speech')

    try:
        speech = rospy.ServiceProxy('speech', Speech)
        speech(String(msg))
    except rospy.ServiceException, e:
        print "Speech failed"

if __name__ == '__main__':
    main()
