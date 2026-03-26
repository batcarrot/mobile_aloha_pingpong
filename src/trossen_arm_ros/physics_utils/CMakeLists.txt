cmake_minimum_required(VERSION 3.10.0)
project(physics_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package into site-packages so it is importable from any node.
ament_python_install_package(${PROJECT_NAME})

ament_package()
