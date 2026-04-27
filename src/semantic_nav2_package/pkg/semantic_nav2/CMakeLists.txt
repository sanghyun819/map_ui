cmake_minimum_required(VERSION 3.8)
project(semantic_nav2)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── Custom service / message definitions ──────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryRoom.srv"
  "srv/QueryObjectsInRoom.srv"
  "srv/QueryNearestObject.srv"
  "msg/SemanticRoom.msg"
  "msg/SemanticObject.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# ── Install Python nodes ───────────────────────────────────────────────────────
install(PROGRAMS
  semantic_nav2/semantic_map_server.py
  semantic_nav2/waypoint_navigator.py
  semantic_nav2/semantic_costmap_layer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
