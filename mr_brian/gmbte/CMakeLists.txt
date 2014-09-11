cmake_minimum_required(VERSION 2.8.3)

find_package(catkin)
project(gmbte)

INCLUDE (FindPkgConfig)

#Controlla se c'e' il programma pkg-config
IF (NOT PKG_CONFIG_FOUND)
	MESSAGE (FATAL_ERROR "pkg-config not found...")
ENDIF (NOT PKG_CONFIG_FOUND)

pkg_check_modules (GTK REQUIRED gtk+-2.0>=2.6)

#Controlla se ci sono le Gtk+-2.0
IF (NOT GTK_FOUND)
	MESSAGE(FATAL_ERROR "You don't seem to have gtk >= 2.6 development libraries installed...")
ENDIF (NOT GTK_FOUND)

ADD_DEFINITIONS()


#Include
INCLUDE_DIRECTORIES (include ${GTK_INCLUDE_DIRS})

#Lincaggio alle librerie
#LINK_DIRECTORIES (${GTK_LIBRARY_DIRS} )
#LINK_LIBRARIES (${GTK_LIBRARIES} )
add_executable(${PROJECT_NAME} src/callbacks.cpp  
		     src/interface.cpp
		     src/main.cpp 
		     src/support.cpp)

target_link_libraries(${PROJECT_NAME} ${GTK_LIBRARIES} brian) 