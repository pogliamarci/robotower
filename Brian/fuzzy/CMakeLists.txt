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
BISON_TARGET(ShapeParser prs/shapegram.y ${PROJECT_SOURCE_DIR}/src/shapegram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/shapegram.tab.h" COMPILE_FLAGS "-p shape")
FLEX_TARGET(ShapeScanner prs/shapeflex.l ${PROJECT_SOURCE_DIR}/src/shapeflex.cpp COMPILE_FLAGS "-Pshape")
BISON_TARGET(AssocParser prs/assocgram.y ${PROJECT_SOURCE_DIR}/src/assocgram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/assocgram.tab.h" COMPILE_FLAGS "-p assoc")
FLEX_TARGET(AssocScanner prs/assocflex.l ${PROJECT_SOURCE_DIR}/src/assocflex.cpp COMPILE_FLAGS "-Passoc")

#aggiunge le dipendenze tra parser e scanner (.l e .y)
ADD_FLEX_BISON_DEPENDENCY(ShapeScanner ShapeParser)
ADD_FLEX_BISON_DEPENDENCY(AssocScanner AssocParser)


#common commands for building c++ executables and libraries
rosbuild_add_library(	fuzzy STATIC
			src/action.cpp
			src/action_list.cpp
			src/action_list_debug.cpp
			src/assoc_file_parser.cpp
			src/association.cpp
			src/association_list.cpp
			src/association_set_multimap.cpp
			src/barycentre_defuzzyfier.cpp
			src/command.cpp
			src/command_list.cpp
			src/command_list_debug.cpp
			src/crisp_data.cpp
			src/crisp_data_list.cpp
			src/crisp_data_list_debug.cpp
			src/data.cpp
			src/defuzzyfier.cpp
			src/div_triangle.cpp
			src/fuzzy_crisp_rel.cpp
			src/fuzzy_data.cpp
			src/fuzzy_data_list.cpp
			src/fuzzy_data_list_debug.cpp
			src/fuzzyfier.cpp
			src/fuzzy_set.cpp
			src/fuzzy_set_multimap.cpp
			src/getFuzzy.cpp
			src/point.cpp
			src/point_list.cpp
			src/point_multimap.cpp
			src/rectangle.cpp
			src/shape.cpp
			src/shape_file_parser.cpp
			src/shape_multimap.cpp
			src/shape_point.cpp
			src/shapes_list.cpp
			src/singleton.cpp
			src/singleton_defuzzyfier.cpp
			src/trapezium.cpp
			src/triangle.cpp
			src/triangle_ol.cpp
			src/triangle_or.cpp
			${BISON_ShapeParser_OUTPUTS}  
                        ${FLEX_ShapeScanner_OUTPUTS}
                        ${BISON_AssocParser_OUTPUTS}  
                        ${FLEX_AssocScanner_OUTPUTS}
		    )

