cmake_minimum_required(VERSION 2.8.3)
project(esp32_ros)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	message_generation
	std_msgs
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS std_msgs message_runtime 
)
