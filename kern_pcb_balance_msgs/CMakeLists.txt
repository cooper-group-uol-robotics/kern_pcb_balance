cmake_minimum_required(VERSION 3.0.2)
project(kern_pcb_balance_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   KernCommand.msg
   KernReading.msg
 )


 generate_messages(
   DEPENDENCIES
std_msgs
 )


catkin_package(
	CATKIN_DEPENDS message_runtime

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)


