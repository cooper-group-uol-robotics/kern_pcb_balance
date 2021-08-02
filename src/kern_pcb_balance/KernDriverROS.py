
#ROS Wrapper for Fisher Scientific PPS4102 Top Pan Balance Serial Driver
#Utilises ROS Topics to facilitate communication with balance using a serial driver
#Made by Jakub Glowacki 27/07/2021

import rospy
from kern_pcb_balance.msg import KernReading
from kern_pcb_balance.msg import KernCommand
from kern_pcb_balance.KernDriverSerial import KernDriver
        
class KernROS:
    
    def __init__(self):
        global pub
        self.kern = KernDriver() #Create object of BalanceDriver class, for serial communication
        #Initialize ros subscriber of topic to which commands are published
        rospy.Subscriber("Kern_Command", KernCommand, self.callback_commands)
        #Initialize ros published for Balance responses (weights)
        pub = rospy.Publisher("Kern_Weights", KernReading, queue_size=10)
        rate = rospy.Rate(10) #Initialize rate object for consistent timed looping
        rospy.loginfo("Kern driver started")
        while not rospy.is_shutdown(): #Whenever driver is running, loop each second polling all values and publishing them to topic
            pub.publish(float(self.kern.weight()))
            rate.sleep()
    
    def zero(self):
        #Simply call upon weight function from driver when correct command is received
        self.kern.zero()
        rospy.loginfo("Zeroing Balance")

        
    def callback_commands(self, msg): #Callback for subscriber. Calls correct function depending on command received
        if(msg.kern_command == msg.ZERO):
        	self.zero()
        else:
        	rospy.loginfo("invalid command")
