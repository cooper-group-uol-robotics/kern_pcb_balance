
#ROS Wrapper for Fisher Scientific PPS4102 Top Pan Balance Serial Driver
#Utilises ROS Topics to facilitate communication with balance using a serial driver
#Made by Jakub Glowacki 27/07/2021

import rospy
from std_msgs.msg import String
from fisherbrand_pps4102_balance.FisherBalanceDriverSerial import BalanceDriver
        
class BalanceDriverROS:
    
    def __init__(self):
        global pub
        self.Balance = BalanceDriver() #Create object of BalanceDriver class, for serial communication
        #Initialize ros subscriber of topic to which commands are published
        rospy.Subscriber("Balance_Commands", String, self.callback_commands)
        #Initialize ros published for Balance responses (weights)
        pub = rospy.Publisher("Balance_Weights", String, queue_size=10)
    
    def weight(self):
        #Simply call upon weight function from driver when correct command is received
        weightStr = self.Balance.weight()
        rospy.loginfo("Printing Stable Weight: " + weightStr + "g")
        pub.publish(weightStr)
        
    def weightNow(self):
        #Same as weight
        weightStr = self.Balance.weightNow()
        rospy.loginfo("Printing Immediate Weight: " + weightStr + "g")
        pub.publish(weightStr)

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
        if(msg.data == "Balance_Off"):
            self.off()
        elif(msg.data == "Balance_On"):
            self.on()
        elif(msg.data == "Balance_Zero"):
            self.zero()
        elif(msg.data == "Balance_Weight"):
            self.weight()
        elif(msg.data == "Balance_WeightNow"):
            self.weightNow()
        else:
           rospy.loginfo("Invalid command")
