cmake_minimum_required(VERSION 3.22)
project(so_arm_moveit LANGUAGES NONE)  # config-only, no compiler needed

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
