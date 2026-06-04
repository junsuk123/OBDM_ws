cmake_minimum_required(VERSION 3.8)
project(moving_platform_sim)

install(DIRECTORY launch models config
  DESTINATION share/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
ament_package()
