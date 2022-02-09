# kern_pcb_balance_driver
#### ROS Driver for Kern PCB Top Pan Balance
#### Uses RS232-USB adapter for serial communication
##### Written by Jakub Glowacki and Hatem Fakhruldeen

## How to Launch
First of all ensure the balance in remote command mode **rE Cr**
The easiest way to launch the package is with roslaunch:
```
roslaunch kern_pcb_balance_driver kern_pcb_driver.launch serial_port:=<port_name>
```
This will launch the plate connected to the provided serial port. If no serial port argument is provided, the default port '/dev/ttyUSB0' will be used.

Alternatively, the balance driver can be launched using rosrun:
```
rosrun kern_pcb_balance_driver kern_pcb_driver <serial_port>
```
Similarly, if no serial port argument is provided, the serial port '/dev/ttyUSB0' is used.

## Balance Serial Settings:
Baud Rate: 9600\
Balance mode: rE Cr

## ROS Topics:
Kern_Commands | For publishing commands to\
Kern_Weights | Where weights are continually published to the scale at approx. 4hz

## Usage:
This scale driver only supports one command, that is to tare the scale down to zero. This command can be triggered as follows:
```
rostopic pub -1 /Kern_Commands kern_pcb_balance/KernCommand "kern_command: 0" 
```
Otherwise, the only functionality of the driver is to continually send the weight currently being read to the Kern_Weights topic as a float32 value "weight". This driver is to be used in conjunction with the peristaltic_dispenser_driver for PID controlled weight based liquid dispensing.
