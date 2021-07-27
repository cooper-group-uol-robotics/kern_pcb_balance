#Script to use the Fisher Scientific PPS4102 Top Pan balance in ROS
#!/usr/bin/env python

import rospy
from std_msgs import String
from fisherbrand_pps4102_balance import ScaleDriverROS
            
if __name__ == "__main__": #Object oriented initialization for ROS
    pub = rospy.Publisher("Scale_Weights", String, queue_size=10) #Initialize publisher
    rospy.init_node("scale_driver", anonymous=True) #initialize rospy node
    scale_ROS = ScaleDriverROS() #Create instance of ROS Wrapper
    scale_ROS.on() #Turn on scale (in case turned off)
    rospy.on_shutdown(scale_ROS.off) #Turn off scale when driver is turned off
    rospy.loginfo("Scale driver started")
    rospy.spin()
