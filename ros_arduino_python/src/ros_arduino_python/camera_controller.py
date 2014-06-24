#!/usr/bin/env python

"""
    A uArm controller class for the Arduino microcontroller
    
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


class CameraController:
    def __init__(self, arduino):

        self.arduino = arduino
        self.rate = float(rospy.get_param("~camera_controller_rate", 10))
        self.timeout = rospy.get_param('~camera_controller_timeout', 1.0)
        self.stopped = False
               
	now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Set up the joint states broadcaster
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState)

        # Camera pos subscriptions
        rospy.Subscriber("/camera/pos", CameraJoints, self.cmdCameraPosCallback)

        # A service to turn a digital sensor on or off
        rospy.Service('/camera/camera_park', CameraPark, self.cmdCameraParkCallback)

        rospy.loginfo("Started Camera controller ")
        
    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            self.t_next = now + self.t_delta

            joints_msg = JointState()
            joints_msg.name = [ 'camera_rot_j', 'camera_tilt_j']
            joints_msg.position = [0.0, 0.0]
            joints_msg.velocity = [0.0, 0.0]
            joints_msg.effort = [0.0, 0.0]
            # Read the servos
            try:
                ServoL, ServoR, ServoROT, ServoHandRot, ServoHand, ServoCameraRot, ServoCameraTilt = self.arduino.get_servos_pos()

            except:
                self.bad_servos_count += 1 
                rospy.logerr("Servos exception count: " + str(self.bad_servos_count))
                return
 
            joints_msg.position[0] = radians(ServoCameraRot) - pi/2                            #JOINT_CAMERA_ROT
            joints_msg.position[1] = radians(ServoCameraTilt) - pi/2                            #JOINT_CAMERA_TILT
           
            joints_msg.header.stamp = rospy.Time.now()
            self.joint_states_pub.publish(joints_msg)

    def cmdCameraPosCallback(self, req):
        print "camera pos callback"
        _camera_tilt = degrees(req.camera_tilt)+90
        _camera_rot = degrees(req.camera_rot)+90

        result = (_camera_tilt < 180)
        result = result & (_camera_tilt > 0)
        result = result & (_camera_rot < 180)
        result = result & (_camera_rot > 0)
        print "result: " + str(result)
        if result == True:
            self.arduino.set_servo_attach(5, 1)
            self.arduino.set_servo_attach(6, 1)

            self.arduino.servo_set_pos(5, _camera_rot)
            self.arduino.servo_set_pos(6, _camera_tilt)

    def cmdCameraParkCallback(self, req):
        if req.value == True: 
            self.cmdCameraPosCallback(CameraJoints(0, 0))
            sleep(1)
            self.arduino.set_servo_attach(5, 0)
            self.arduino.set_servo_attach(6, 0)

        else:
            self.arduino.set_servo_attach(5, 1)
            self.arduino.set_servo_attach(6, 1)



    

    
