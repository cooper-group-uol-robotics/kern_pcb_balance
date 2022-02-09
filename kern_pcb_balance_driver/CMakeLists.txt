cmake_minimum_required(VERSION 3.0.2)
project(kern_pcb_balance_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  kern_pcb_balance_msgs
)


catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS kern_pcb_balance_msgs

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS nodes/kern_pcb_driver
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


