cmake_minimum_required(VERSION 3.0.2)
project(baxter_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  kdl_parser
  baxter_core_msgs
  baxter_interface_cpp
)

find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES baxter_kinematics
  CATKIN_DEPENDS roscpp urdf kdl_parser baxter_core_msgs baxter_interface_cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/baxter_kinematics.cpp
  src/baxter_ik_client.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

add_executable(test_baxter_kinematics_node src/test_kinematics_node.cpp)
target_link_libraries(test_baxter_kinematics_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
