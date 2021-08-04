#!/usr/bin/env python

#Serial Driver for Kern PCB Top Pan balance
#Uses USB-Serial-RS232 communication to send commands and receive messages
#Made by Jakub Glowacki 27/07/2021

import time
import serial
import re

class KernDriver:
    serialCom = serial.Serial() #Globally define serial communication
    
    def __init__(self): #Init function starts serial communication
        global serialCom 
        serialCom = serial.Serial( #Initialize serial communication object
            port='/dev/ttyUSB0',
            baudrate = 9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.2
            )
        
    #Commands are defined in balance manual, however need to be sent over serial as
    #ASCII encoded byte arrays and must end with a carriage return and line break to
    #be recognized. Received messsages can also be decoded then to unicode strings.

    def weight(self):
        #Read and return weight being sent over serial by balance
        global serialCom 
        x=serialCom.read_until("\n") #Serial read
        stringx=str(x.decode('ascii')) #convert to string
        #use regular expression to get float weight value
        s = re.findall("[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", stringx) 
        if (len(s) > 0): #return value if it exists, otherwise return 0
        	return s[0]
        else:
        	return 0
    
    def zero(self):
        #Zero out the balance
        global serialCom
        serialCom.write(bytearray("t\r\n", "ascii")) #Send Zero (Tare) Command
        return True

 
