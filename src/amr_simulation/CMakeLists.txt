cmake_minimum_required(VERSION 3.8)
project(amr_simulation)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Generate before both copy-install and symlink-install. A plain colcon build
# must never silently install worlds compiled from an earlier configuration.
add_custom_target(generate_warehouse ALL
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_warehouse.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Regenerating deterministic warehouse scenarios")
install(DIRECTORY config launch models worlds scripts
  DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(warehouse_generation tests/test_generation.py TIMEOUT 180)
  ament_add_pytest_test(warehouse_launch tests/test_launch.py TIMEOUT 180)
endif()
ament_package()
