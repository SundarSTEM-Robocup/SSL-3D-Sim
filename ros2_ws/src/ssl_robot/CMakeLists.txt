cmake_minimum_required(VERSION 3.8)
project(ssl_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install URDF and Meshes
install(
  DIRECTORY urdf meshes launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Scripts (The UI Controller)
install(
    PROGRAMS src/gui_controller.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
