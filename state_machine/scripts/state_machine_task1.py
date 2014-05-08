#!/usr/bin/env python

import rospy
import smach
import smach_ros
import os
from std_msgs.msg import Empty, Int32, Bool, String, Float64MultiArray
from wait_button.msg import button
from dora_nav_goals.msg import named_coordinates

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

		say('Goodbye, Doctor')
		rospy.sleep(1)
		
		return 'Spoken'

class NavigateDoor(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Succeeded','Failed'])
		self.run_pub = rospy.Publisher("/navigate/request", named_coordinates)
		self.result_sub = rospy.Subscriber("/navigate/response", Bool, self.result_cb)

	def execute(self, userdata):
		log('In state NAVIGATE_DOOR')

		self.executed = False
		
		msg = named_coordinates()
		msg.pos_name = 'Door'
		msg.goal.target_pose.header.frame_id = '/map'
		msg.goal.target_pose.header.stamp = rospy.get_rostime()
		msg.goal.target_pose.pose.position.x = -3.1
                msg.goal.target_pose.pose.position.y = -0.96
                msg.goal.target_pose.pose.position.z = 0
                msg.goal.target_pose.pose.orientation.x = 0
   		msg.goal.target_pose.pose.orientation.y = 0
   	        msg.goal.target_pose.pose.orientation.z = -0.80
    		msg.goal.target_pose.pose.orientation.w = 0.59

		self.run_pub.publish(msg)

		while not rospy.is_shutdown():
			if self.executed:
				if self.succeeded:
					return 'Succeeded'
				else:
					return 'Failed'
			else:
				rospy.sleep(0.2)

	def result_cb(self, msg):
		self.succeeded = msg.data
		self.executed = True
		
class NavigateEnter(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Succeeded_Follow','Succeeded_End','Failed'])
		self.run_pub = rospy.Publisher("/navigate/request", String)
		self.result_sub = rospy.Subscriber("/navigate/response", Bool, self.result_cb)

	def execute(self, userdata):
		log('In state NAVIGATE_ENTER')

		self.executed = False

		msg = named_coordinates()
		msg.pos_name = 'Enter'
		msg.goal.target_pose.header.frame_id = '/map'
		msg.goal.target_pose.header.stamp = rospy.get_rostime()
		msg.goal.target_pose.pose.position.x = -2.2
		msg.goal.target_pose.pose.position.y = 0.28
		msg.goal.target_pose.pose.position.z = 0
		msg.goal.target_pose.pose.orientation.x = 0
		msg.goal.target_pose.pose.orientation.y = 0
		msg.goal.target_pose.pose.orientation.z = -0.85
		msg.goal.target_pose.pose.orientation.w = 0.52

		self.run_pub.publish(msg)

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
		self.succeeded = msg.data
		self.executed = True
		
class NavigateBedroom(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Succeeded','Failed'])
		self.run_pub = rospy.Publisher("/navigate/request", String)
		self.result_sub = rospy.Subscriber("/navigate/response", Bool, self.result_cb)

	def execute(self, userdata):
		log('In state NAVIGATE_BEDROOM')

		self.executed = False

		msg = named_coordinates()
		msg.pos_name = 'Bedroom'
		msg.goal.target_pose.header.frame_id = '/map'
		msg.goal.target_pose.header.stamp = rospy.get_rostime()
		msg.goal.target_pose.pose.position.x = 2.18
		msg.goal.target_pose.pose.position.y = -1.26
		msg.goal.target_pose.pose.position.z = 0
		msg.goal.target_pose.pose.orientation.x = 0
		msg.goal.target_pose.pose.orientation.y = 0
		msg.goal.target_pose.pose.orientation.z = -0.75
		msg.goal.target_pose.pose.orientation.w = 0.65

		self.run_pub.publish(msg)

		while not rospy.is_shutdown():
			if self.executed:
				if self.succeeded:
					return 'Succeeded'
				else:
					return 'Failed'
			else:
				rospy.sleep(0.2)

	def result_cb(self, msg):
		self.succeeded = msg.data
		self.executed = True
		
class Teleop(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['True_Door','True_Enter','False'])
		self.result_sub = rospy.Subscriber("/wait_for_button", button, self.result_cb)

	def execute(self, userdata):
		log('In state TELEOP')

		self.executed = False

		while not rospy.is_shutdown():
			if self.executed:
				if self.buttons.button1 == 1:
					return 'True_Door'
				elif self.buttons.button2 == 1:
					return 'True_Enter'
				else:
					return 'False'
			else:
				rospy.sleep(0.01)

	def result_cb(self, msg):
		self.buttons = msg
		self.executed = True
		
class FaceRecognition(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Doctor','Unknown'])
		self.run_pub = rospy.Publisher("/face_recognition/request", Empty)
		self.result_sub = rospy.Subscriber("/face_recognizer_ID", String, self.result_cb)

	def execute(self, userdata):
		log('In state FACE_RECOGNITION')

		self.executed = False
		
		self.run_pub.publish()

		while not rospy.is_shutdown():
			if self.executed:
				if self.person == 'Doctor':
					return self.person
				else:
					return 'Unknown'
			else:
				rospy.sleep(0.2)

	def result_cb(self, msg):
		self.person = msg.data
		self.executed = True
	
def main():
	rospy.init_node('state_machine')

	sm = smach.StateMachine(outcomes=['END'])

	with sm:
		smach.StateMachine.add('TELEOP', Teleop(), transitions={'True_Door':'NAVIGATE_DOOR','True_Enter':'SPEECH_LEAVE_BEDROOM','False':'TELEOP'})
		smach.StateMachine.add('NAVIGATE_DOOR', NavigateDoor(), transitions={'Succeeded':'FACE_RECOGNITION','Failed':'NAVIGATE_DOOR'})
		smach.StateMachine.add('NAVIGATE_ENTER', NavigateEnter(), transitions={'Succeeded_Follow':'SPEECH_FOLLOW_ME','Succeeded_End':'SPEECH_END','Failed':'NAVIGATE_ENTER'})
		smach.StateMachine.add('NAVIGATE_BEDROOM', NavigateBedroom(), transitions={'Succeeded':'TELEOP','Failed':'NAVIGATE_BEDROOM'})
		smach.StateMachine.add('FACE_RECOGNITION', FaceRecognition(), transitions={'Doctor':'SPEECH_DOCTOR','Unknown':'FACE_RECOGNITION'})
		smach.StateMachine.add('SPEECH_LEAVE_BEDROOM', SpeechLeaveBedroom(), transitions={'Spoken':'NAVIGATE_ENTER'})
		smach.StateMachine.add('SPEECH_DOCTOR', SpeechDoctor(), transitions={'Spoken':'NAVIGATE_ENTER'})
		smach.StateMachine.add('SPEECH_FOLLOW_ME', SpeechFollowMe(), transitions={'Spoken':'NAVIGATE_BEDROOM'})
		smach.StateMachine.add('SPEECH_END', SpeechEnd(), transitions={'Spoken':'TELEOP'})
		
	rospy.sleep(2)

	outcome = sm.execute()

def log(msg):
	rospy.loginfo(msg)
	
def say(msg):
	os.system( 'espeak -ven+f3 \"' + msg + '\"' ); # Voice synthetizer
	log('Saying \"' + msg + '\"')
	speech_pub.publish(msg)


if __name__ == '__main__':
	main()
