cmake_minimum_required(VERSION 3.0.2)
project(baxter_mpc_controller)

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
  baxter_core_msgs
  baxter_interface_cpp
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  dynamic_reconfigure
  kdl_parser
  urdf
)

find_package(casadi REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/MpcParameters.cfg
)

catkin_install_python(PROGRAMS
  scripts/enable_simulated_baxter.py
  scripts/record_control_diagnostics.py
  scripts/send_sim_references.py
  scripts/pick_and_place_demo.py
  scripts/enforce_arm_gravity.py
  scripts/summarize_control_diagnostics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS baxter_core_msgs baxter_interface_cpp roscpp std_msgs sensor_msgs visualization_msgs dynamic_reconfigure kdl_parser urdf
 DEPENDS casadi orocos_kdl EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/mpc_controller.cpp
  src/mpc_controller_node.cpp
  src/collision_primitives.cpp
  src/arm_collision_model.cpp
  src/geometry_rows.cpp
  src/joint_state_listener.cpp
  src/geometry_markers.cpp
  include/mpc_controller/mpc_controller.hpp
  include/mpc_controller/mpc_controller_node.hpp
  include/mpc_controller/collision_primitives.hpp
  include/mpc_controller/arm_collision_model.hpp
  include/mpc_controller/geometry_rows.hpp
  include/mpc_controller/joint_state_listener.hpp
  include/mpc_controller/geometry_markers.hpp
)

add_executable(run_mpc src/main.cpp)
add_executable(mpc_solver_benchmark src/mpc_solver_benchmark.cpp)

target_link_libraries(run_mpc PUBLIC
  baxter_interface_cpp
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(mpc_solver_benchmark PUBLIC
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# The library includes the generated dynamic_reconfigure header, so it must wait for gencfg
# explicitly (parallel clean builds otherwise race it).
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(run_mpc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mpc_solver_benchmark ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
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

install(TARGETS mpc_solver_benchmark
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.rviz"
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(collision_primitives_test test/collision_primitives_test.cpp)
  target_link_libraries(collision_primitives_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(arm_collision_model_test
    test/arm_collision_model.test
    test/arm_collision_model_test.cpp
  )
  target_link_libraries(arm_collision_model_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
  add_rostest_gtest(geometry_rows_test
    test/geometry_rows.test
    test/geometry_rows_test.cpp
  )
  target_link_libraries(geometry_rows_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
  add_rostest_gtest(mpc_geometry_rows_test
    test/mpc_geometry_rows.test
    test/mpc_geometry_rows_test.cpp
  )
  target_link_libraries(mpc_geometry_rows_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} casadi)
endif()
