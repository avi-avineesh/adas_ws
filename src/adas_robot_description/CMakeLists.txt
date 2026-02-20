cmake_minimum_required(VERSION 3.8)
project(adas_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
