cmake_minimum_required(VERSION 3.8)
project(uav_worlds)

install(DIRECTORY worlds models wind
  DESTINATION share/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
ament_package()
