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

#cerca i package Bison e Flex
find_package(BISON)
find_package(FLEX)

#genera i parser .cpp dai sorgenti flex e Bison
BISON_TARGET(PredParser prs/predgram.y ${PROJECT_SOURCE_DIR}/src/predgram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/predgram.tab.h" COMPILE_FLAGS "-p pred")
FLEX_TARGET(PredScanner prs/predflex.l ${PROJECT_SOURCE_DIR}/src/predflex.cpp COMPILE_FLAGS "-Ppred")
BISON_TARGET(RulesParser prs/rulesgram.y ${PROJECT_SOURCE_DIR}/src/rulesgram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/rulesgram.tab.h"  COMPILE_FLAGS "-p rules")
FLEX_TARGET(RulesScanner prs/rulesflex.l  ${PROJECT_SOURCE_DIR}/src/rulesflex.cpp COMPILE_FLAGS "-Prules")

#aggiunge le dipendenze tra parser e scanner (.l e .y)
ADD_FLEX_BISON_DEPENDENCY(PredScanner PredParser)
ADD_FLEX_BISON_DEPENDENCY(RulesScanner RulesParser)

#FILE(GLOB HEADERS "*.tab.hpp")
#FILE(REMOVE HEADERSTOREMOVE ${HEADERS})

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
			${FLEX_PredScanner_OUTPUTS}
			${BISON_PredParser_OUTPUTS}  
			${BISON_RulesParser_OUTPUTS}
		 	${FLEX_RulesScanner_OUTPUTS}
			${FLEX_RulesScanner_OUTPUTS}
		    )

