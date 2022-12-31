cmake_minimum_required(VERSION 2.8.3)

project(exoskeleton_btp)

# find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 std_msgs
 actionlib
 actionlib_msgs
 message_generation
 geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
	FILES
	imuData.msg
	hallState.msg
	velocityEffort.msg
	exoJointVelocity.msg
	exoJointCommand.msg
	jointFeedback.msg
	walkingParameters.msg
	exoState.msg
	exoStates.msg
	imuArray.msg
	imuQuaternion.msg
	exoJointFeedbackMsg.msg
	exoJointCommand.msg
	exoPid.msg
	exoAllJointFeedbackMsg.msg
	exoAllJointCommand.msg
	exoAllJointTrajectory.msg
	exoAllPid.msg
	uiDisplayMessage.msg
	userInput.msg
	batteryInfo.msg
)

# ## Generate services in the 'srv' folder
# add_service_files(
#  FILES
#  demo_srv.srv
# )

# ## Generate actions in the 'action' folder
# add_action_files(
#  FILES
#  Demo_action.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 actionlib_msgs
 geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs message_runtime geometry_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

#Demo action server
# add_executable(demo_action_server src/demo_action_server.cpp)
# add_dependencies(demo_action_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(demo_action_server ${catkin_LIBRARIES} )

# #Demo action client
# add_executable(demo_action_client src/demo_action_client.cpp)
# add_dependencies(demo_action_client  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(demo_action_client ${catkin_LIBRARIES})
