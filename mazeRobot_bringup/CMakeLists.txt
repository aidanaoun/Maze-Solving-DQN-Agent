cmake_minimum_required(VERSION 3.8)
project(mazeRobot_bringup)

find_package(ament_cmake REQUIRED)

# Install resources that the launch files will load
install(DIRECTORY launch urdf params models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

