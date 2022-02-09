#!/usr/bin/env python

#ROS Wrapper for KERN PCB Top Pan Balance Serial Driver
#Utilises ROS Topics to facilitate communication with balance using a serial driver
#Made by Jakub Glowacki 02/08/2021

import rospy
from kern_pcb_balance_msgs.msg import KernReading
from kern_pcb_balance_msgs.msg import KernCommand
from kern_pcb_balance_driver.kern_serial_driver import KernDriver
        
class KernROS:
    
    def __init__(self, serial_port):
        global pub
        global zero
        zero = False
        self.kern = KernDriver(serial_port) #Create object of KernDriver class, for serial communication
        #Initialize ros subscriber of topic to which commands are published
        rospy.Subscriber("Kern_Commands", KernCommand, self.callback_commands)
        #Initialize ros published for balance responses (weights)
        pub = rospy.Publisher("Kern_Weights", KernReading, queue_size=10)
        rate = rospy.Rate(10) #Initialize rate object for consistent timed looping
        rospy.loginfo("Kern driver started")
        while not rospy.is_shutdown(): #Whenever driver is running, loop each second polling all values and publishing them to topic
            if (not zero):
                pub.publish(float(self.kern.weight()))
            else:
                pub.publish(float(-1))
            rate.sleep()
    
    def zero(self):
        #Simply call upon weight function from driver when correct command is received
        global zero
        zero=True
        rospy.sleep(2)
        self.kern.zero()
        rospy.loginfo("Zeroing Balance")
        rospy.sleep(2)
        zero=False

        
    def callback_commands(self, msg): #Callback for subscriber. Calls correct function depending on command received
        if(msg.kern_command == msg.ZERO):
        	self.zero()
        else:
        	rospy.loginfo("invalid command")
