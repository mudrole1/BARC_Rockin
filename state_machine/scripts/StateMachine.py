#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty, Int32, Bool, String, Float64MultiArray

speech_pub = rospy.Publisher("/say", String)

class SpeechLeaveBedroom(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Spoken'])

	def execute(self, userdata):
		log('In state SPEECH_LEAVE_BEDROOM')

		NavigateEnter.outcome = 'End'

		say('Thank you, Doctor')
		rospy.sleep(1)
		
		return 'Spoken'
	
class SpeechDoctor(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Spoken'])

	def execute(self, userdata):
		log('In state SPEECH_DOCTOR')

		NavigateEnter.outcome = 'Follow'

		say('Hello, Doctor')
		rospy.sleep(1)
		
		return 'Spoken'
	
class SpeechFollowMe(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Spoken'])

	def execute(self, userdata):
		log('In state SPEECH_FOLLOW_ME')

		say('Please follow me to the bedroom')
		rospy.sleep(1)
		
		return 'Spoken'
	
class SpeechEnd(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Spoken'])

	def execute(self, userdata):
		log('In state SPEECH_END')

		say('lalalalala')
		rospy.sleep(1)
		
		return 'Spoken'

class NavigateDoor(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Succeeded','Failed'])
		self.run_pub = rospy.Publisher("/navigate/request", String)
		self.result_sub = rospy.Subscriber("/navigate/response", String, self.result_cb)

	def execute(self, userdata):
		log('In state NAVIGATE_DOOR')

		self.executed = False

		self.run_pub.publish('Door')

		while not rospy.is_shutdown():
			if self.executed:
				if self.succeeded:
					return 'Succeeded'
				else:
					return 'Failed'
			else:
				rospy.sleep(0.2)

	def result_cb(self, msg):
		self.succeeded = msg
		self.executed = True
		
class NavigateEnter(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Succeeded_Follow','Succeeded_End','Failed'])
		self.run_pub = rospy.Publisher("/navigate/request", String)
		self.result_sub = rospy.Subscriber("/navigate/response", String, self.result_cb)

	def execute(self, userdata):
		log('In state NAVIGATE_ENTER')

		self.executed = False

		self.run_pub.publish('Enter')

		while not rospy.is_shutdown():
			if self.executed:
				if self.succeeded:
					if NavigateEnter.outcome == 'Follow':
						return 'Succeeded_Follow'
					elif NavigateEnter.outcome == 'End':
						return 'Succeeded_End'
					else: log('Invalid NAVIGATE_ENTER outcome: ' + NavigateEnter.outcome)
				else:
					return 'Failed'
			else:
				rospy.sleep(0.2)

	def result_cb(self, msg):
		self.succeeded = msg
		self.executed = True
		
class NavigateBedroom(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Succeeded','Failed'])
		self.run_pub = rospy.Publisher("/navigate/request", String)
		self.result_sub = rospy.Subscriber("/navigate/response", String, self.result_cb)

	def execute(self, userdata):
		log('In state NAVIGATE_BEDROOM')

		self.executed = False

		self.run_pub.publish('Bedroom')

		while not rospy.is_shutdown():
			if self.executed:
				if self.succeeded:
					return 'Succeeded'
				else:
					return 'Failed'
			else:
				rospy.sleep(0.2)

	def result_cb(self, msg):
		self.succeeded = msg
		self.executed = True
		
class Teleop(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['True_Door','True_Enter','False'])
		self.result_sub = rospy.Subscriber("/teleop/response", String, self.result_cb)

	def execute(self, userdata):
		log('In state TELEOP')

		self.executed = False

		while not rospy.is_shutdown():
			if self.executed:
				if self.destination == 'Door':
					return 'True_Door'
				elif self.destination == 'Enter':
					return 'True_Enter'
				else:
					return 'False'
			else:
				rospy.sleep(0.2)

	def result_cb(self, msg):
		self.destination = msg
		self.executed = True
		
class FaceRecognition(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Doctor'])
		self.run_pub = rospy.Publisher("/face_recognition/request", Empty)
		self.result_sub = rospy.Subscriber("/face_recognition/response", String, self.result_cb)

	def execute(self, userdata):
		log('In state FACE_RECOGNITION')

		self.executed = False
		
		self.run_pub.publish()

		while not rospy.is_shutdown():
			if self.executed:
				if self.person == 'Doctor':
					return self.person
			else:
				rospy.sleep(0.2)

	def result_cb(self, msg):
		self.person = msg
		self.executed = True
	
def main():
	rospy.init_node('state_machine')

	sm = smach.StateMachine(outcomes=['END'])

	with sm:
		smach.StateMachine.add('TELEOP', Teleop(), transitions={'True_Door':'NAVIGATE_DOOR','True_Enter':'SPEECH_LEAVE_BEDROOM','False':'TELEOP'})
		smach.StateMachine.add('NAVIGATE_DOOR', NavigateDoor(), transitions={'Succeeded':'FACE_RECOGNITION','Failed':'NAVIGATE_DOOR'})
		smach.StateMachine.add('NAVIGATE_ENTER', NavigateEnter(), transitions={'Succeeded_Follow':'SPEECH_FOLLOW_ME','Succeeded_End':'SPEECH_END','Failed':'NAVIGATE_ENTER'})
		smach.StateMachine.add('NAVIGATE_BEDROOM', NavigateBedroom(), transitions={'Succeeded':'TELEOP','Failed':'NAVIGATE_BEDROOM'})
		smach.StateMachine.add('FACE_RECOGNITION', FaceRecognition(), transitions={'Doctor':'SPEECH_DOCTOR'})
		smach.StateMachine.add('SPEECH_LEAVE_BEDROOM', SpeechLeaveBedroom(), transitions={'Spoken':'NAVIGATE_ENTER'})
		smach.StateMachine.add('SPEECH_DOCTOR', SpeechDoctor(), transitions={'Spoken':'NAVIGATE_ENTER'})
		smach.StateMachine.add('SPEECH_FOLLOW_ME', SpeechFollowMe(), transitions={'Spoken':'NAVIGATE_BEDROOM'})
		smach.StateMachine.add('SPEECH_END', SpeechEnd(), transitions={'Spoken':'TELEOP'})
		
	rospy.sleep(2)

	outcome = sm.execute()

def log(msg):
	rospy.loginfo(msg)
	
def say(msg):
	log('Saying ' + msg)
	speech_pub.publish(msg)

if __name__ == '__main__':
	main()
