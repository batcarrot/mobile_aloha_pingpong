cmake_minimum_required(VERSION 3.10.0)
project(mpc_ros)

find_package(ament_cmake REQUIRED)

install(DIRECTORY mpc_ros DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/mpc_node.py
  scripts/ball_prediction_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
