#!/usr/bin/env python

import roslib; roslib.load_manifest('tom_smach')
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import Bool
from std_msgs.msg import String


# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to wait command'])
        self.counter = 0
        self.state_start_time = rospy.Time.now()
        self.voice_command = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
        self.voice_command = 0
        now = rospy.Time.now()
        self.state_start_time = now
        self.state_end_time = self.state_start_time + rospy.Duration(10.0)
        # Voice command subscriptions
        rospy.Subscriber('recognizer/output', String, self.speechCb)


        while self.voice_command == 0:
            now = rospy.Time.now()
            #print str(now)
        self.voice_command == 0
        return 'to wait command'

    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        if msg.data.find("move") > -1:
            self.voice_command = 1


# define state WaitCommand
class WaitCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to idle'])
        self.state_start_time = rospy.Time.now()
        # subscriptions

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitCommand')
        now = rospy.Time.now()
        self.state_start_time = now
        self.state_end_time = self.state_start_time + rospy.Duration(10.0)
        gripper_pub = rospy.Publisher('/gripper', Bool)
        gripper_pub.publish(Bool(True))

        while now < self.state_end_time:
            now = rospy.Time.now()
            #print str(now)
        gripper_pub.publish(Bool(False))
        return 'to idle'
        



# main
def main():
    rospy.init_node('tom_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])


    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'to wait command':'WAIT COMMAND'})
        smach.StateMachine.add('WAIT COMMAND', WaitCommand(), 
                               transitions={'to idle':'IDLE'})
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
