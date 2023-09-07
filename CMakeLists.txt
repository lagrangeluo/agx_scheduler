cmake_minimum_required(VERSION 3.0.2)

project(agx_scheduler)

#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif()

#include(GNUInstallDirs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
)

find_package(rmf_traffic REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)


add_message_files(
  FILES
  Waypoint.msg
  SchedulePath.msg
)

add_service_files(
  FILES
  add_waypoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  message_generation
)

include_directories(
  include
  include/${PROJECT_NAME}
)
 
  add_executable(agx_scheduler_node
    src/agx_scheduler_node.cpp
    src/parse_graph.cpp
  )
  target_link_libraries(agx_scheduler_node
    ${catkin_LIBRARIES}
    rmf_traffic::rmf_traffic
    yaml-cpp
    ${rmf_utils_LIBRARIES}
    ${Eigen3_LIBRARIES}
  )
 add_dependencies(agx_scheduler_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
 
  target_include_directories(agx_scheduler_node
    PRIVATE
      ${rmf_traffic_INCLUDE_DIRS}
      ${Eigen3_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
  )

 install(TARGETS agx_scheduler_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
  install(
    TARGETS
      agx_scheduler_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  install(DIRECTORY launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   )