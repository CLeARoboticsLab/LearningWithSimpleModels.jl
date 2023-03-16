cmake_minimum_required(VERSION 3.0.2)
project(lwsm_jetracer)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
)

add_message_files(
  FILES
  RolloutData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(controller
	src/controller.cpp
)
target_link_libraries(
	controller
	${catkin_LIBRARIES}
)

install(TARGETS	controller
  	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)