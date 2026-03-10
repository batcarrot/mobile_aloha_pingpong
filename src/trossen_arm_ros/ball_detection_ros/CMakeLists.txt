cmake_minimum_required(VERSION 3.10.0)
project(ball_detection_ros)

find_package(ament_cmake REQUIRED)

install(DIRECTORY ball_detection_ros DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ball_position_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
