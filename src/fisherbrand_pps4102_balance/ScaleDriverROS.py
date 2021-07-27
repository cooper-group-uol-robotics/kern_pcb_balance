#ROS Wrapper for Fisher Scientific PPS4102 Top Pan Balance Serial Driver
#Utilises ROS Topics to facilitate communication with balance using a serial driver
#Made by Jakub Glowacki 27/07/2021

import rospy
from std_msgs import String
from fisherbrand_pps4102_balance import ScaleDriver
        
class ScaleDriverROS:
    
    def __init__(self):
        global pub
        self.scale = ScaleDriver() #Create object of ScaleDriver class, for serial communication
        #Initialize ros subscriber of topic to which commands are published
        rospy.Subscriber("Scale_Commands", String, self.callback_commands)
        #Initialize ros published for scale responses (weights)
        pub = rospy.Publisher("Scale_Weights", String, queue_size=10)
    
    def weight(self):
        #Simply call upon weight function from driver when correct command is received
        weightStr = self.scale.weight()
        rospy.loginfo("Printing Stable Weight: " + weightStr + "g")
        pub.publish(weightStr)
        
    def weightNow(self):
        #Same as weight
        weightStr = self.scale.weightNow()
        rospy.loginfo("Printing Immediate Weight: " + weightStr + "g")
        pub.publish(weightStr)

        #Call upon appropriate function in driver for any possible command
    def zero(self):
        self.scale.zero()
        rospy.loginfo("Zeroing Scale")

    def off(self):
        self.scale.off()
        rospy.loginfo("Scale going into standby")
        
    def on(self):
        self.scale.on()
        rospy.loginfo("Scale turning on")
        
    def callback_commands(self, msg): #Callback for subscriber. Calls correct function depending on command received
        if(msg.data == "Scale_Off"):
            self.off()
        elif(msg.data == "Scale_On"):
            self.on()
        elif(msg.data == "Scale_Zero"):
            self.zero()
        elif(msg.data == "Scale_Weight"):
            self.weight()
        elif(msg.data == "Scale_WeightNow"):
            self.weightNow()
        else:
           rospy.loginfo("Invalid command")