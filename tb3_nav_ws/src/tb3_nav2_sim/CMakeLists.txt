cmake_minimum_required(VERSION 3.8)
project(tb3_nav2_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds maps params
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/send_goal.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
