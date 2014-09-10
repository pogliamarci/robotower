cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS  roscpp
					 roslib
					 message_generation)

find_package(OpenCV REQUIRED)

add_message_files(  
  DIRECTORY msg
  FILES Results.msg
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include 
		    lib
		    ${catkin_INCLUDE_DIRS})

#Build the node
add_executable(${PROJECT_NAME} src/Blob.cpp 
			       src/BlobBuffer.cpp 
			       src/ColorClassifier.cpp 
			       src/ColorDataset.cpp 
			       src/DistanceCalculator.cpp 
			       src/ImageAnalyzer.cpp 
			       src/PixelMap.cpp 
			       src/Receptor.cpp 
			       src/VisionParameters.cpp 
			       src/main.cpp)
			       
#Add ANN
ADD_LIBRARY(ANN STATIC IMPORTED)
SET_TARGET_PROPERTIES(ANN PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libANN.a)

target_link_libraries(${PROJECT_NAME} ANN ${OpenCV_LIBS} ${catkin_LIBRARIES})
			       
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
