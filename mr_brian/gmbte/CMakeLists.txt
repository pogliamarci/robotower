cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS mr_brian)
project(gmbte)

if(catkin_FOUND)
  catkin_package()
endif()

include(FindPkgConfig)

#Controlla se c'e' il programma pkg-config
if (NOT PKG_CONFIG_FOUND)
	message (FATAL_ERROR "pkg-config not found...")
endif (NOT PKG_CONFIG_FOUND)

pkg_check_modules (GTK REQUIRED gtk+-2.0>=2.6)

#Controlla se ci sono le Gtk+-2.0
if (NOT GTK_FOUND)
	message(FATAL_ERROR "You don't seem to have gtk >= 2.6 development libraries installed...")
endif (NOT GTK_FOUND)

add_definitions()

if(catkin_FOUND)
include_directories(include ${GTK_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
else()
#TODO include brian & gtk
#include_directories(include ${GTK_INCLUDE_DIRS} 
endif()


add_executable(${PROJECT_NAME} src/callbacks.cpp  
			       src/interface.cpp
			       src/main.cpp 
			       src/support.cpp)
		     
		     
if(catkin_FOUND)

target_link_libraries(${PROJECT_NAME} ${GTK_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
else()
#TODO link brian
endif()