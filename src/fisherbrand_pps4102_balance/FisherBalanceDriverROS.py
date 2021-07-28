
#ROS Wrapper for Fisher Scientific PPS4102 Top Pan Balance Serial Driver
#Utilises ROS Topics to facilitate communication with balance using a serial driver
#Made by Jakub Glowacki 27/07/2021

import rospy
from fisherbrand_pps4102_balance.msg import BalanceReading
from fisherbrand_pps4102_balance.msg import BalanceCommand
from fisherbrand_pps4102_balance.FisherBalanceDriverSerial import BalanceDriver
        
class BalanceDriverROS:
    
    def __init__(self):
        global pub
        self.Balance = BalanceDriver() #Create object of BalanceDriver class, for serial communication
        #Initialize ros subscriber of topic to which commands are published
        rospy.Subscriber("Balance_Commands", BalanceCommand, self.callback_commands)
        #Initialize ros published for Balance responses (weights)
        pub = rospy.Publisher("Balance_Weights", BalanceReading, queue_size=10)
    
    def weight(self):
        #Simply call upon weight function from driver when correct command is received
        weightStr = self.Balance.weight()
        rospy.loginfo("Printing Stable Weight: " + weightStr + "g")
        pub.publish(float(weightStr))
        
    def weightNow(self):
        #Same as weight
        weightStr = self.Balance.weightNow()
        rospy.loginfo("Printing Immediate Weight: " + weightStr + "g")
        pub.publish(float(weightStr))

        #Call upon appropriate function in driver for any possible command
    def zero(self):
        self.Balance.zero()
        rospy.loginfo("Zeroing Balance")

    def off(self):
        self.Balance.off()
        rospy.loginfo("Balance going into standby")
        
    def on(self):
        self.Balance.on()
        rospy.loginfo("Balance turning on")
        
    def callback_commands(self, msg): #Callback for subscriber. Calls correct function depending on command received
        if(msg.balance_command == msg.ZERO):
        	self.zero()
        elif(msg.balance_command == msg.BALANCE_ON):
        	self.on()
        elif(msg.balance_command == msg.BALANCE_OFF):
        	self.off()
        elif(msg.balance_command == msg.WEIGHT_STABLE):
        	self.weight()
        elif(msg.balance_command == msg.WEIGHT_NOW):
        	self.weightNow()
        else:
        	rospy.loginfo("invalid command")
