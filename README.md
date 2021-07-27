# fisherbrand_pps4102_balance
ROS Driver for Fisher Scientific PPS4102 Top Pan Balance
Uses RS232-USB adapter for serial communication
Written by Jakub Glowacki

Scale Settings:
Baud Rate: 9600
Parity: None
Bits: 8
No Handshake

ROS Topics:
Scale_Commands | For publishing commands to
Scale_Weights | Where weights will be published to from the scale

Possible commands:
Scale_Off | Turns scale to standby mode
Scale_On | Turns scale back on from standby
Scale_Zero | Re-calibrates (Tares) the scale back to zero
Scale_Weight | Gets the weight currently read by the scale, waits until weight is stabilized then publishes to Scale_Weights
Scale_WeightNow | Gets the weight read by the scale immediately without waiting for stability, publishes to Scale_Weights
