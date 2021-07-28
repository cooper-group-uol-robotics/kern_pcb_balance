# fisherbrand_pps4102_balance
#### ROS Driver for Fisher Scientific PPS4102 Top Pan Balance
#### Uses RS232-USB adapter for serial communication
##### Written by Jakub Glowacki

## How to Launch
The easiest way to launch the package is with roslaunch:
```
roslaunch fisherbrand_pps4102_balance FisherROS.launch
```
Alternatively, can be launched using rosrun:
```
rosrun fisherbrand_pps4102_balance FisherBalanceROS
```

## Balance Serial Settings:
Baud Rate: 9600\
Parity: None\
Bits: 8\
No Handshake

## ROS Topics:
Balance_Commands | For publishing commands to\
Balance_Weights | Where weights will be published to from the scale

## How to send Commands:
Commands are sent using the Balance_Commands topic. Each command has its own command ID which can be sent through the topic as a balance_command. A balance_command is just a simple integer corresponding to a command, as indicated by the list below. For example, to zero the scale use the command:
```
rostopic pub -1 /Balance_Commands fisherbrand_pps4102_balance/BalanceCommand "balance_command: 0" 

```

## Possible commands:
0 | Re-calibrates (Tares) the balance back to zero\
1 | Turns balance back on from standby\
2 | Turns balance to standby mode\
3 | Gets the weight currently read by the balance, waits until weight is stabilized then publishes to Balance_Weights\
4 | Gets the weight read by the balance immediately without waiting for stability, publishes to Balance_Weights
