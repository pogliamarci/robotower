cmake_minimum_required(VERSION 2.8.3)
project(spykee)

find_package(catkin REQUIRED COMPONENTS  roscpp
					 message_generation
                                         sensor_msgs)
add_message_files(  
  DIRECTORY msg
  FILES Motion.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime
                              sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

#Build the node
add_executable(${PROJECT_NAME} src/spykee.cpp 
			       src/SpykeeManager.cpp 
			       src/PracticalSocket.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
			       
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

