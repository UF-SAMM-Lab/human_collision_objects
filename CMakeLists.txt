cmake_minimum_required(VERSION 3.0.2)
project(human_collision_objects)
add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
#add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
  geometry_msgs
  visualization_msgs
  subscription_notifier
  moveit_msgs
  moveit_ros_planning_interface
  rosparam_utilities
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES human_collision_objects
  CATKIN_DEPENDS moveit_core roscpp geometry_msgs visualization_msgs subscription_notifier moveit_ros_planning_interface rosparam_utilities
  DEPENDS
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/utilities.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(sim_human src/fake_human.cpp)
target_link_libraries(sim_human
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
add_dependencies(sim_human ${catkin_EXPORTED_TARGETS})
install(TARGETS sim_human DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(live_human src/live_human.cpp)
target_link_libraries(live_human
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
add_dependencies(live_human ${catkin_EXPORTED_TARGETS})
install(TARGETS live_human DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
