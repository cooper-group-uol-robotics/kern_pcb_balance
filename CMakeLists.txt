cmake_minimum_required(VERSION 3.0.2)
project(fisherbrand_pps4102_balance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)


catkin_python_setup()

<<<<<<< HEAD
=======

>>>>>>> ed584f5798acedc26a6446e665cf28e676dac7ef
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
<<<<<<< HEAD

include_directories(
=======

)


>>>>>>> ed584f5798acedc26a6446e665cf28e676dac7ef
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS nodes/FisherBalanceROS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


