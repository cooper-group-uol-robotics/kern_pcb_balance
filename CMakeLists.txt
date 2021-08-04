cmake_minimum_required(VERSION 3.0.2)
project(fisherbrand_pps4102_balance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)


catkin_python_setup()

 add_message_files(
   FILES
   BalanceCommand.msg
   BalanceReading.msg
 )


 generate_messages(
   DEPENDENCIES
std_msgs
 )


catkin_package(
	CATKIN_DEPENDS message_runtime

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS nodes/FisherBalanceROS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


