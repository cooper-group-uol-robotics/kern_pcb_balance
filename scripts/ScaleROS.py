#Serial Driver for Fisher Scientific PPS4102 Top Pan balance
#Uses USB-Serial-RS232 communication to send commands and receive messages
#Made by Jakub Glowacki 27/07/2021

#!/usr/bin/env python

import rospy
# from ScaleDriver import ScaleDriver #Scale serial driver imported, will be called to
from std_msgs.msg import String #String message type utilized
import time
import serial
import re

class ScaleDriver:
    serialCom = serial.Serial() #Globally define serial communication
    
    def __init__(self): #Init function starts serial communication
        global serialCom 
        serialCom = serial.Serial( #Initialize serial communication object
            port='/dev/ttyUSB0',
            baudrate = 9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS,
            timeout=1
            )
        
    #Commands are defined in Balance manual, however need to be sent over serial as
    #ASCII encoded byte arrays and must end with a carriage return and line break to
    #be recognized. Received messsages can also be decoded then to unicode strings.
        
    def weight(self):
        #Print a stable weight, waits until weight is stably detected
        global serialCom
        serialCom.write(bytearray("P\r\n", "ascii")) #Write command for stable weight print
        x=serialCom.read_until("\n") #Read response
        stringx=str(x.decode('ascii')) #Decode response
        s = re.findall("[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", stringx) #Use Regex to extract only
                                                                                       #numbers
        while (len(s) < 2): #Repeat process until stable weight is found, otherwise error may occur due to
                            #weight number not existing
            time.sleep(1)
            x=serialCom.read_until("\n")
            stringx=str(x.decode('ascii'))
            s = re.findall("[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", stringx)
        return s[1]

    def weightNow(self):
        #Immediately print weight on scale, regardless of stability
        #Identical to weight function but uses IP instead of P command and doesn't wait for stability
        global serialCom
        serialCom.write(bytearray("IP\r\n", "ascii"))
        x=serialCom.read_until("\n")
        stringx=str(x.decode('ascii'))
        s = re.findall("[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", stringx) 
        return s[1]
    
    def zero(self):
        #Zero out the scale
        global serialCom
        serialCom.write(bytearray("T\r\n", "ascii")) #Send Zero (Tare) Command
        return True

    def off(self):
        #Turn off scale
        global serialCom
        serialCom.write(bytearray("OFF\r\n", "ascii"))
        return True
        
    def on(self):
        #Turn on Scale
        global serialCom
        serialCom.write(bytearray("ON\r\n", "ascii"))
        return True
        
#ROS Wrapper for Fisher Scientific PPS4102 Top Pan Balance Serial Driver
#Utilises ROS Topics to facilitate communication with balance using a serial driver
#Made by Jakub Glowacki 27/07/2021
        
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
            
if __name__ == "__main__": #Object oriented initialization for ROS
    pub = rospy.Publisher("Scale_Weights", String, queue_size=10) #Initialize publisher
    rospy.init_node("scale_driver", anonymous=True) #initialize rospy node
    scale_ROS = ScaleDriverROS() #Create instance of ROS Wrapper
    scale_ROS.on() #Turn on scale (in case turned off)
    rospy.on_shutdown(scale_ROS.off) #Turn off scale when driver is turned off
    rospy.loginfo("Scale driver started")
    rospy.spin()
