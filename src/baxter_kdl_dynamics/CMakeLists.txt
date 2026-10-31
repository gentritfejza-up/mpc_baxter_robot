cmake_minimum_required(VERSION 3.0.2)
project(baxter_kdl_dynamics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

find_package(catkin REQUIRED COMPONENTS
  baxter_dynamics
  baxter_interface_cpp
  kdl_parser
  roscpp
  std_msgs
  urdf
)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS baxter_dynamics baxter_interface_cpp kdl_parser roscpp std_msgs urdf
  DEPENDS orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# The KDL model only; the torque loop comes from baxter_dynamics.
add_library(${PROJECT_NAME} SHARED
  src/kdl_inverse_dynamics.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

add_executable(run_kdl_inverse_dynamics
  src/kdl_inverse_dynamics_node.cpp
)
target_link_libraries(run_kdl_inverse_dynamics
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} run_kdl_inverse_dynamics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
