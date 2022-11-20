cmake_minimum_required(VERSION 3.0.0)
project(geodetic_utils)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# find_package(Eigen REQUIRED)
set(EIGEN3_INCLUDE_DIR "/home/ubuntu/3rdparty/eigen-3.3.4")
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
# include_directories(${Eigen_INCLUDE_DIRS})

cs_add_executable(set_gps_reference_node src/set_gps_reference_node.cpp)
target_link_libraries(set_gps_reference_node  ${catkin_LIBRARIES})

cs_add_executable(gps_to_pose_conversion_node src/gps_to_pose_conversion_node.cpp)
#target_link_libraries(gps_to_pose_conversion_node  ${catkin_LIBRARIES})

cs_install()
cs_install_scripts(src/euler_radians_to_degrees.py)
cs_export()
