cmake_minimum_required(VERSION 3.10.2)
project(robotiq_85_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
