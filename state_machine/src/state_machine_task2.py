#!/usr/bin/env python

import rospy
import smach
import smach_ros
import os
from std_msgs.msg import Empty, Int32, Bool, String, Float64MultiArray

speech_pub = rospy.Publisher("/say", String)

DEBUG = 0

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
            self.result = 1
            self.executed = True

        self.run_pub.publish(Navigation.destination)

        while not rospy.is_shutdown():
            if self.executed:
                if self.result == 1:
                    return Navigation.next_state
                elif self.result == 0:
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
        smach.State.__init__(self, outcomes=['Doctor','Deliman','Postman','Unknown','No Face'])
        self.run_pub = rospy.Publisher("/recognition/request", Empty)
        self.result_sub = rospy.Subscriber("/recognition/response", String, self.result_cb)

    def execute(self, userdata):
        log('In state RECOGNITION')

        self.executed = False
		
        self.run_pub.publish()

        while not rospy.is_shutdown():
            if self.executed:
                if self.person in ('Doctor','Deliman','Postman','Unknown'):
                    return self.person
                elif self.person == 'No Face':
                    Speech.msg = FACE_CAMERA_MESSAGE
                    Speech.next_state = 'RECOGNITION'
                    return self.person
                else:
                    return 'Unknown'
            else:
                rospy.sleep(0.2)

    def result_cb(self, msg):
        self.person = msg.data
        self.executed = True

class Speech(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['NAVIGATION','RECOGNITION','LISTEN','POSTMAN_ENTERED','POSTMAN_WAIT','DELIMAN_ENTERED','DELIMAN_WAIT','DOCTOR_ENTERED','DOCTOR_WAIT'])

    def execute(self, userdata):
        log('In state SPEECH. Message: ' + Speech.msg)

        say(Speech.msg)
        rospy.sleep(1)
		
        return Speech.next_state

class Doctor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['NAVIGATION'])

    def execute(self, userdata):
        log('Doctor detected')

        Navigation.destination = 'Enter'
        Navigation.next_state = 'SPEECH'
        Speech.msg = DOCTOR_HELLO_MESSAGE
        Speech.next_state = 'DOCTOR_ENTERED'
		
        return 'NAVIGATION'

class Deliman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['NAVIGATION'])

    def execute(self, userdata):
        log('Deliman detected')

        Navigation.destination = 'Enter'
        Navigation.next_state = 'SPEECH'
        Speech.msg = DELIMAN_HELLO_MESSAGE
        Speech.next_state = 'DELIMAN_ENTERED'
		
        return 'NAVIGATION'

class Postman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['NAVIGATION'])

    def execute(self, userdata):
        log('Postman detected')

        Navigation.destination = 'Enter'
        Navigation.next_state = 'SPEECH'
        Speech.msg = POSTMAN_HELLO_MESSAGE
        Speech.next_state = 'POSTMAN_ENTERED'
		
        return 'NAVIGATION'

class UnknownPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):
        log('Unknown person detected')

        Speech.msg = UNKNOWN_GO_AWAY_MESSAGE
        Speech.next_state = 'LISTEN'
		
        return 'Speech'

class PostmanEntered(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):

        Speech.msg = POSTMAN_LEAVE_POST_MESSAGE
        Speech.next_state = 'POSTMAN_WAIT'
		
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

        Speech.msg = FOLLOW_MESSAGE
        Speech.next_state = 'NAVIGATION'
        Navigation.destination = 'Kitchen'
        Navigation.next_state = 'DELIMAN_IN_KITCHEN'
		
        return 'Speech'

class DelimanInKitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):

        Speech.msg = DELIMAN_BREAKFAST_MESSAGE
        Speech.next_state = 'DELIMAN_WAIT'
		
        return 'Speech'

class DelimanWait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Leaving'])
        self.run_pub = rospy.Publisher("/wait_for_deliman/request", Empty)
        self.result_sub = rospy.Subscriber("/wait_for_deliman/response", Empty, self.result_cb)

    def execute(self, userdata):
        self.executed = False

        run_pub.publish()

        if DEBUG:
            self.executed = True

        while not rospy.is_shutdown():
            if self.executed:
                Speech.msg = FOLLOW_MESSAGE
                Speech.next_state = 'NAVIGATION'
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

        Speech.msg = FOLLOW_MESSAGE
        Speech.next_state = 'NAVIGATION'
        Navigation.destination = 'Bedroom'
        Navigation.next_state = 'DOCTOR_IN_BEDROOM'
		
        return 'Speech'

class DoctorInBedroom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Speech'])

    def execute(self, userdata):

        Speech.msg = DOCTOR_WAIT_MESSAGE
        Speech.next_state = 'DOCTOR_WAIT'
		
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
                Speech.msg = FOLLOW_MESSAGE
                Speech.next_state = 'NAVIGATION'
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

        Speech.msg = BYE_MESSAGE
        Speech.next_state = 'LISTEN'
		
        return 'Speech'
	
def main():
    rospy.init_node('state_machine')

    sm = smach.StateMachine(outcomes=['END'])

    with sm:
        smach.StateMachine.add('LISTEN', Listen(), transitions={'Bell':'NAVIGATION'})
        smach.StateMachine.add('NAVIGATION', Navigation(), transitions={'SPEECH':'SPEECH','RECOGNITION':'RECOGNITION','DELIMAN_IN_KITCHEN':'DELIMAN_IN_KITCHEN','DOCTOR_IN_BEDROOM':'DOCTOR_IN_BEDROOM','BYE':'BYE','Failed':'NAVIGATION','Unknown':'END'})
        smach.StateMachine.add('RECOGNITION', Recognition(), transitions={'Doctor':'DOCTOR','Deliman':'DELIMAN','Postman':'POSTMAN','Unknown':'UNKNOWN_PERSON','No Face':'SPEECH'})
        smach.StateMachine.add('SPEECH', Speech(), transitions={'NAVIGATION':'NAVIGATION','RECOGNITION':'RECOGNITION','LISTEN':'LISTEN','POSTMAN_ENTERED':'POSTMAN_ENTERED','POSTMAN_WAIT':'POSTMAN_WAIT','DELIMAN_ENTERED':'DELIMAN_ENTERED','DELIMAN_WAIT':'DELIMAN_WAIT','DOCTOR_ENTERED':'DOCTOR_ENTERED','DOCTOR_WAIT':'DOCTOR_WAIT'})
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

    outcome = sm.execute()

def log(msg):
    rospy.loginfo(msg)
	
def say(msg):
    log('Saying \"' + msg + '\"')
#    os.system( 'espeak -ven+f3 \"' + msg + '\"' ); # Voice synthetizer (previous)
    os.system( 'espeak --stdout \"' + msg + '\" | paplay' ); # Voice synthetizer (changed by Sean)
    speech_pub.publish(msg)


if __name__ == '__main__':
    main()
