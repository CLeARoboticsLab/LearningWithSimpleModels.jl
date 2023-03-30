cmake_minimum_required(VERSION 3.0.2)
project(lwsm_quadruped)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	ros_sockets
	unitree_legged_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH})

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

add_executable(quad_controller
	src/quad_controller.cpp
)
target_link_libraries(
	quad_controller
	${catkin_LIBRARIES}
)

add_executable(test_quad_control
	src/test_quad_control.cpp
)
target_link_libraries(
	test_quad_control
	${catkin_LIBRARIES}
)

add_executable(quad_simulation
	src/quad_simulation.cpp
)
target_link_libraries(
	quad_simulation
	${catkin_LIBRARIES}
)

install(TARGETS	quad_controller test_quad_control quad_simulation
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