cmake_minimum_required(VERSION 3.8)
project(amr_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config urdf meshes launch rviz worlds scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
ament_package()
