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
			src/assocflex.cpp
			src/shapegram.tab.cpp
			src/shapeflex.cpp
			src/assocgram.tab.cpp

		    )

#target_link_libraries(fuzzy libfuzzy.a)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(isaac src/isaac.cpp)
#target_link_libraries(example ${PROJECT_NAME})
