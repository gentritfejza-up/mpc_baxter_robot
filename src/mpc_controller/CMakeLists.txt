cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

# Enable compiler optimizations and export compile commands for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

find_package(dynamic_reconfigure REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  baxter_kinematics
  baxter_core_msgs
  baxter_interface_cpp
  roscpp
  std_msgs
  dynamic_reconfigure
)

find_package(casadi REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/MpcParameters.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mpc_controller
 CATKIN_DEPENDS baxter_kinematics baxter_core_msgs baxter_interface_cpp roscpp std_msgs dynamic_reconfigure
 DEPENDS casadi
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/mpc_controller.cpp
  src/mpc_controller_node.cpp
  include/mpc_controller/mpc_controller.hpp
  include/mpc_controller/mpc_controller_node.hpp
)

add_executable(run_mpc src/main.cpp)

target_link_libraries(run_mpc PUBLIC
  baxter_interface_cpp
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  casadi
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS run_mpc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

