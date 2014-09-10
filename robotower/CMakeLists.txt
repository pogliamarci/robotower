cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp 
					echoes)
project(robotower)

# If catkin is not found, just compile the executable without ROS goodies
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Qt4 COMPONENTS QtCore QtXml QtGui REQUIRED)
include(${QT_USE_FILE})
# This is necessary as all ui files etc will get dumped in the bottom of the
# binary directory.
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${QT_QTXML_INCLUDE_DIR})
include_directories("${QT_INCLUDE_DIR}/../QtMobility")
include_directories("${QT_INCLUDE_DIR}/../QtMultimediaKit")
include_directories("${QT_INCLUDE_DIR}/../QtMultimedia")

set(QT_USE_QTXML TRUE)
set(QT_USE_QTMULTIMEDIA TRUE)
set(QT_LIBRARIES "${QT_LIBRARIES};-lQtMultimediaKit")

##############################################################################
# Sections
##############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

