cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp
					mr_brian)

project(isaac)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#Build the node
add_executable(${PROJECT_NAME} src/Isaac.cpp 
			       src/BlockedTimeCalculator.cpp 
			       src/SensorStatus.cpp 
			       src/Sender.cpp 
			       src/IsaacStrategy.cpp)


