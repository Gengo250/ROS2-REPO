cmake_minimum_required(VERSION 3.8)
project(amr_lite_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EntityState.msg" "msg/Obstacle.msg" "msg/ObstacleArray.msg" "msg/SimEvent.msg" "msg/WorldState.msg"
  "srv/AddObstacle.srv" "srv/RemoveObstacle.srv"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
