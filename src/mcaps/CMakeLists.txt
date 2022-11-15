cmake_minimum_required(VERSION 2.8.3)
project(flourish_mapping)

SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
#set(CMAKE_AUTOMOC ON)

add_compile_options(-fPIC)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DRELEASE")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DRELEASE")
ELSE()
  MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

find_package(G2O)
find_package(SuiteSparse REQUIRED)

# FIND_PACKAGE(CSparse)
# IF(CSPARSE_FOUND)
#   SET(BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
# ELSE(CSPARSE_FOUND)
#   SET(BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
#   IF(BUILD_CSPARSE)
#     SET(CSPARSE_FOUND TRUE)
#   ENDIF()
# ENDIF(CSPARSE_FOUND)

###############################################################################
## Catkin
###############################################################################
#add_definitions(-Wall)

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp cv_bridge image_transport sensor_msgs std_msgs rosbag pose_graph velodyne_msgs geodetic_utils
                                        )
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Xml DBus)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(Qt5Core REQUIRED)
  find_package(Qt5Gui REQUIRED)
  find_package(Qt5Network REQUIRED)
  find_package(Qt5PrintSupport REQUIRED)
  find_package(Qt5Sql REQUIRED)
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5Xml REQUIRED)
  find_package(Qt5LinguistTools)

#add OpenMP support
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES flourish_mapping_lib
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs rosbag velodyne_msgs
  DEPENDS PCL
)

#include(${QT_USE_FILE})

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  include
  ${PCL_INCLUDE_DIRS}
  $ENV{G2O_ROOT}
  ${CSPARSE_INCLUDE_DIR}
)



# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} forms/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp include/*.h)

##############################################################################
# Binaries
##############################################################################
#add_library(flourish_mapping_lib
#  ${QT_SOURCES}
#  ${QT_RESOURCES_CPP}
#  ${QT_FORMS_HPP}
#  ${QT_MOC_HPP}
#)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(flourish_mapping
  ${QT_SOURCES}
  ${QT_RESOURCES_CPP}
  ${QT_FORMS_HPP}
  ${QT_MOC_HPP}
)
target_link_libraries(flourish_mapping
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libcxsparse.so.3.1.4
  $ENV{G2O_ROOT}/lib/libg2o_core.so
  $ENV{G2O_ROOT}/lib/libg2o_stuff.so
  $ENV{G2O_ROOT}/lib/libg2o_csparse_extension.so
  $ENV{G2O_ROOT}/lib/libg2o_types_slam2d.so
  $ENV{G2O_ROOT}/lib/libg2o_types_slam3d.so
  $ENV{G2O_ROOT}/lib/libg2o_solver_csparse.so
  yaml-cpp
)

