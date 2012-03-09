cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(	brian STATIC 
			src/brian.cpp 
			src/behavior_eng.cpp 
			src/can_doer.cpp 
			src/compose.cpp 	
			src/preacher.cpp 
			src/predicate.cpp 
			src/predicate_list.cpp 
			src/predicate_list_debug.cpp
			src/proposed_action_list.cpp
			src/proposed_action_list_debug.cpp
			src/wanter_eng.cpp
			src/weight_want.cpp
			src/weight_want_list.cpp
			src/weight_want_list_debug.cpp
			src/engine_objects.cpp 
			src/rules_behav.cpp
			src/rulesflex.cpp
			src/predgram.tab.cpp
			src/predflex.cpp
			src/rulesgram.tab.cpp
		    )

#target_link_libraries(fuzzy libfuzzy.a)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(isaac src/isaac.cpp)
#target_link_libraries(example ${PROJECT_NAME})
