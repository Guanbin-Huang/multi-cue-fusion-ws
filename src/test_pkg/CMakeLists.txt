
cmake_minimum_required(VERSION 3.0.0)
project(test_pkg)


SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
#set(CMAKE_AUTOMOC ON)

add_compile_options(-fPIC)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic -g")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
# opencv

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

set(G2O_ROOT "/home/ubuntu/3rdparty")
find_package(SuiteSparse REQUIRED)
set(EIGEN3_INCLUDE_DIR "/home/ubuntu/3rdparty/eigen-3.3.4")


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  include
  $ENV{G2O_ROOT}
  ${CSPARSE_INCLUDE_DIR}
)

INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )


add_executable(test_pkg test_pkg.cpp
)

target_link_libraries(test_pkg
  ${catkin_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libcxsparse.so.3.1.4
  /home/ubuntu/3rdparty/g2o-20170730/lib/libg2o_core.so
  /home/ubuntu/3rdparty/g2o-20170730/lib/libg2o_stuff.so
  /home/ubuntu/3rdparty/g2o-20170730/lib/libg2o_csparse_extension.so
  /home/ubuntu/3rdparty/g2o-20170730/lib/libg2o_types_slam2d.so
  /home/ubuntu/3rdparty/g2o-20170730/lib/libg2o_types_slam3d.so
  /home/ubuntu/3rdparty/g2o-20170730/lib/libg2o_solver_csparse.so
)

