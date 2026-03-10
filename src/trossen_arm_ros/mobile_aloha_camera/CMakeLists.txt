cmake_minimum_required(VERSION 3.10.0)
project(mobile_aloha_camera)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_flir_dual.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
