cmake_minimum_required(VERSION 2.8.3)
project(echoes)

find_package(catkin REQUIRED COMPONENTS  roscpp
					 roslib
					 message_generation)


add_message_files(DIRECTORY msg
		  FILES Rfid.msg  Sonar.msg  Towers.msg)

add_service_files(DIRECTORY srv
		  FILES BlinkingLed.srv  FixedLed.srv  ResetLed.srv)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

#Build the node
add_executable(${PROJECT_NAME} src/SerialCommunication.cpp 
			       src/CharCircularBuffer.cpp 
			       src/main.cpp src/Dispatcher.cpp 
			       src/Processer.cpp 
			       src/RfidProcesser.cpp 
			       src/SonarProcesser.cpp 
			       src/TowerProcesser.cpp 
			       src/LedParser.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
			       
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
