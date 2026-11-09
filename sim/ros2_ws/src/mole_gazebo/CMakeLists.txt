cmake_minimum_required(VERSION 3.8)
project(mole_gazebo NONE)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/smoke_test.py scripts/gen_model.py scripts/extract_params.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
