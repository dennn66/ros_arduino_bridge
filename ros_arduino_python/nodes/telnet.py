#!/usr/bin/env python
import roslib
import rospy
import sys
import getpass
import sys
import telnetlib

HOST = "192.168.1.120"

tn = telnetlib.Telnet(HOST, 2000)
#tn.read_until("login: ")
tn.write("hello world\r\n")
tn.write("ls\r\n")
tn.write("exit\r\n")

# Initalize ROS
rospy.init_node('telnet_emu')

while not rospy.is_shutdown():
    print "."
    print tn.read_until("\r")

    #print tn.read_all()
