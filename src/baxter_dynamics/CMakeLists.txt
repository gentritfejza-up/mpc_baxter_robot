cmake_minimum_required(VERSION 3.0.2)
project(baxter_dynamics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

find_package(catkin REQUIRED COMPONENTS
  baxter_interface_cpp
  roscpp
  std_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS baxter_interface_cpp roscpp std_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Dynamics models (generic interface + Lagrange-Euler), the computed-torque law
# and the backend-independent torque loop.
add_library(${PROJECT_NAME} SHARED
  src/arm_description.cpp
  src/forward_kinematics.cpp
  src/lagrange_euler.cpp
  src/computed_torque_law.cpp
  src/torque_loop.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(run_lagrange_euler
  src/lagrange_euler_main.cpp
)

target_link_libraries(run_lagrange_euler PUBLIC
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} run_lagrange_euler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  # Closed-form checks on synthetic arms (planar RR, prismatic); no ROS master needed.
  catkin_add_gtest(planar_arm_test test/planar_arm_test.cpp)
  target_link_libraries(planar_arm_test ${PROJECT_NAME} ${catkin_LIBRARIES})

  # Baxter DH model against KDL built from the live URDF.
  find_package(rostest REQUIRED)
  find_package(kdl_parser REQUIRED)
  find_package(urdf REQUIRED)
  find_package(orocos_kdl REQUIRED)
  add_rostest_gtest(lagrange_euler_kdl_test
    test/lagrange_euler_kdl.test
    test/lagrange_euler_kdl_test.cpp
  )
  target_include_directories(lagrange_euler_kdl_test PRIVATE
    ${kdl_parser_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
  )
  target_link_libraries(lagrange_euler_kdl_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${urdf_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
  )
endif()
