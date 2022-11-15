cmake_minimum_required(VERSION 3.0.0)
project(pose_graph)

add_definitions(-std=c++11)

set(OpenCV_DIR "/opt/ros/kinetic/include/opencv-3.3.1-dev")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  roslib
  rosbag
  cv_bridge
)

find_package(Boost REQUIRED)
#find_package(yaml-cpp REQUIRED)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tss_lib
)

find_package(OpenCV 3 REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
 # ${YAML_CPP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

#link_directories(${Boost_LIBRARY_DIR})

add_library( tss_lib src/library/temporalnode.cpp src/library/temporalgraph.cpp)
target_link_libraries(tss_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(posegraphnode src/nodes/pose_graph_node_bonirob.cpp)
#add_dependencies(posegraphnode ${catkin_EXPORTED_TARGETS})
#target_link_libraries(posegraphnode tss_lib ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})
set(YAML_CPP_LIBRARIES yaml-cpp)

add_executable(build_graph src/nodes/build_sync_graph.cpp)
# add_executable(build_graph src/nodes/build_graph_generic.cpp)
#add_executable(build_graph src/nodes/build_graph_eschikon.cpp)
add_dependencies(build_graph ${catkin_EXPORTED_TARGETS})
target_link_libraries(build_graph tss_lib ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})

add_executable(reduce_graph src/nodes/reduce_graph.cpp)
add_dependencies(reduce_graph ${catkin_EXPORTED_TARGETS})
target_link_libraries(reduce_graph tss_lib ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})

add_executable(extract_image_registration_data src/nodes/extract_image_registration_data.cpp)
add_dependencies(extract_image_registration_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(extract_image_registration_data tss_lib ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mergeSequentialBagsExample src/nodes/mergeSequentialBagsExample.cpp)
add_dependencies(mergeSequentialBagsExample ${catkin_EXPORTED_TARGETS})
target_link_libraries(mergeSequentialBagsExample tss_lib ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})


add_executable(mergeBagsExample src/nodes/mergeBags.cpp)
add_dependencies(mergeBagsExample ${catkin_EXPORTED_TARGETS})
target_link_libraries(mergeBagsExample tss_lib ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})
