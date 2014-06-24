#!/usr/bin/env python

"""
    A uArm controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os
#from dynamic_reconfigure.server import Server
#from ros_arduino_python.cfg import PIDConfig

from math import sin, cos, pi, atan2, radians, degrees, acos, atan, sqrt
#from geometry_msgs.msg import Quaternion, Twist, Pose
#from nav_msgs.msg import Odometry
#from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import JointState
from ros_arduino_msgs.msg import *
from std_msgs.msg import Bool
from time import sleep
from ros_arduino_msgs.srv import *


#from ros_arduino_python.covariances import ODOM_POSE_COVARIANCE, ODOM_TWIST_COVARIANCE


uArmJointNames = [
  'base_body_j',
  'body_upper_arm_j',
  'upper_arm_forearm_j',
  'forearm_wrist_j',
  'wrist_palm_j',
  'palm_left_finger_j',
  'palm_right_finger_j']

 
""" Class to receive Twist commands and publish Odometry data """
class UarmController:
    def __init__(self, arduino):
        self.GRIPPER_OFFSET_X = 35
        self.GRIPPER_OFFSET_Z = 100
        self.FIXED_OFFSET_R = 30.0
        self.FIXED_OFFSET_L = 45.0

        self.arduino = arduino
        self.rate = float(rospy.get_param("~uarm_controller_rate", 10))
        self.timeout = rospy.get_param('~uarm_controller_timeout', 1.0)
        self.stopped = False
               
	now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Set up the joint states broadcaster
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState)

        # Gripper subscriptions
        rospy.Subscriber("/uarm/gripper", Bool, self.cmdGripperCallback)

        # uArm pos subscriptions
        rospy.Subscriber("/uarm/uarm_pos", Joints, self.cmdUarmPosCallback)

        # A service to turn a digital sensor on or off
        rospy.Service('/uarm/uarm_park', UarmPark, self.cmdUarmParkCallback)


        rospy.loginfo("Started uArm controller ")
        
    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            self.t_next = now + self.t_delta

            joints_msg = JointState()
            joints_msg.name = ['base_body_j',
		  'body_upper_arm_j',
		  'upper_arm_forearm_j',
		  'forearm_wrist_j',
		  'wrist_palm_j',
		  'palm_left_finger_j',
		  'palm_right_finger_j']
            joints_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joints_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joints_msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # Read the servos
            try:
                ServoL, ServoR, ServoROT, ServoHandRot, ServoHand, ServoCameraRot, ServoCameraTilt = self.arduino.get_servos_pos()

            except:
                self.bad_servos_count += 1 
                rospy.logerr("Servos exception count: " + str(self.bad_servos_count))
                return
            #define FIXED_OFFSET_L          18 0.314
            #define FIXED_OFFSET_R          36 0.628

            joints_msg.position[0] = radians(ServoROT) - pi/2                             #JOINT_BASE_BODY
            joints_msg.position[1] = pi - (radians(ServoL)+0.624)                         #JOINT_BODY_UPPER_ARM
            joints_msg.position[2] = (radians(ServoL)+0.624) + (radians(ServoR)+2.246)    #JOINT_UPPER_ARM_FOREARM
            joints_msg.position[3] = pi - (radians(ServoR)+2.246)                         #JOINT_FOREARM_WRIST
            joints_msg.position[4] = radians(ServoHandRot) - pi/2                         #JOINT_WRIST_PALM
            joints_msg.position[5] = radians(55) -radians(ServoHandRot)                   #JOINT_PALM_RIGHT_FINGER
            joints_msg.position[6] = radians(ServoHandRot)-radians(55)                    #JOINT_PALM_LEFT_FINGER

	    #	for joint in self.joint_states.values():
	    #	    msg.name.append(joint.name)
	    #	    msg.position.append(joint.position)
	    #	    msg.velocity.append(joint.velocity)
	    #	    msg.effort.append(joint.effort)
           
            joints_msg.header.stamp = rospy.Time.now()
            self.joint_states_pub.publish(joints_msg)
            	
    def cmdGripperCallback(self, req):
        # Handle gripper catch/release requests
        self.arduino.set_servo_attach(4, 1)
        print str(req)
        if req == Bool(True): 
            self.arduino.servo_set_pos(4, 55)
        else:
            self.arduino.servo_set_pos(4, 17)

    def cmdUarmPosCallback(self, req):

        # Handle gripper catch/release requests
	_armRot = -req.arm_rot
	# input limit
	_stretch = req.stretch 	- self.GRIPPER_OFFSET_X	# 
	_height  = req.height 	+ self.GRIPPER_OFFSET_Z
	_armRot  = _armRot + 90.0		# +90, change -90~90 to 0~180
	_handRot = req.hand_rot + 90.0	# +90, change -90~90 to 0~180
	# angle calculation
	stretch2height2 = _stretch * _stretch + _height * _height 
        ARM_A       =            148.0    # upper arm
        ARM_B       =            160.0    # lower arm
        ARM_2AB     =            47360.0  # 2*A*B
        ARM_A2      =            21904.0  # A^2
        ARM_B2      =            25600.0  # B^2
        ARM_A2B2    =            47504.0  # A^2 + B^2
        T = sqrt(stretch2height2)
        #print "T: " + str(T)
        result = (T < ARM_A+ARM_B)
        result = result & (T > 55)
	a = degrees((acos( (ARM_A2B2 - stretch2height2) / ARM_2AB ))) # angle between the upper and the lower
	if _stretch == 0 :
            b = 90
        else:
            b = degrees(atan(_height/_stretch)) 
        #print b
	c = degrees((acos((ARM_A2 + stretch2height2 -ARM_B2)/(2 * ARM_A * T))))
	d = 180 - a - c
        angleBs = d - b
        result = result & (angleBs > -6)
        angleAs = c+b
        #print "angleAs: " + str(angleAs)
        #print "angleBs: " + str(angleBs)

        result = result & (angleAs > 0)
        result = result & (angleAs+15 < 180-angleBs)

	angleR = angleBs + self.FIXED_OFFSET_R
	angleL = angleAs + self.FIXED_OFFSET_L
	# angle limit
        result = result & (angleL < 180)
        result = result & (angleL > 0)
        result = result & (angleR < 180)
        result = result & (angleR > 0)
        #print "result: " + str(result)
        if result == True:
            self.arduino.set_servo_attach(0, 1)
            self.arduino.set_servo_attach(1, 1)
            self.arduino.set_servo_attach(2, 1)
            self.arduino.set_servo_attach(3, 1)

            self.arduino.servo_set_pos(0, angleL)
            self.arduino.servo_set_pos(1, angleR)
            self.arduino.servo_set_pos(2, _armRot)
            self.arduino.servo_set_pos(3, _handRot)

    def cmdUarmParkCallback(self, req):
        if req.value == True: 
            self.arduino.set_servo_attach(2, 0)
            self.arduino.set_servo_attach(3, 0)
            self.cmdUarmPosCallback(Joints(160, 0, 0, 0))
            sleep(1)
            self.cmdUarmPosCallback(Joints(80, -20, 0, 0))
            sleep(1)
            self.arduino.set_servo_attach(0, 0)
            self.arduino.set_servo_attach(1, 0)

        else:
            self.arduino.set_servo_attach(0, 1)
            self.arduino.set_servo_attach(1, 1)
            self.arduino.set_servo_attach(2, 1)
            self.arduino.set_servo_attach(3, 1)


    

    
