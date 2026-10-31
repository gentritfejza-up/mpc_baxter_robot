cmake_minimum_required(VERSION 3.0.2)
project(simulator_demo)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/enable_simulated_baxter.py
  scripts/enforce_arm_gravity.py
  scripts/send_sim_references.py
  scripts/pick_and_place_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
