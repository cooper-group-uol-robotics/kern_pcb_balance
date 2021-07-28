#!/usr/bin/env python

#Serial Driver for Fisher Scientific PPS4102 Top Pan balance
#Uses USB-Serial-RS232 communication to send commands and receive messages
#Made by Jakub Glowacki 27/07/2021

import time
import serial
import re

class BalanceDriver:
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
        #Immediately print weight on Balance, regardless of stability
        #Identical to weight function but uses IP instead of P command and doesn't wait for stability
        global serialCom
        serialCom.write(bytearray("IP\r\n", "ascii"))
        x=serialCom.read_until("\n")
        stringx=str(x.decode('ascii'))
        s = re.findall("[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", stringx) 
        return s[1]
    
    def zero(self):
        #Zero out the Balance
        global serialCom
        serialCom.write(bytearray("T\r\n", "ascii")) #Send Zero (Tare) Command
        return True

    def off(self):
        #Turn off Balance
        global serialCom
        serialCom.write(bytearray("OFF\r\n", "ascii"))
        return True
        
    def on(self):
        #Turn on Balance
        global serialCom
        serialCom.write(bytearray("ON\r\n", "ascii"))
        return True
