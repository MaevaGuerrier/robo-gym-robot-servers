cmake_minimum_required(VERSION 3.8)
project(simulation_objects)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/objects_controller.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()