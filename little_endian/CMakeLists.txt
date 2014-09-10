cmake_minimum_required(VERSION 2.8.3)
project(little_endian)

find_package(catkin REQUIRED COMPONENTS roscpp
					spykee)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#Build the node
add_executable(${PROJECT_NAME} src/littleendian.cpp src/Zone.cpp src/LittleObject.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
