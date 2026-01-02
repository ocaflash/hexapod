cmake_minimum_required(VERSION 3.8)
project(hexapod_brain)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hexapod_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

set(SOURCES
  src/requester/coordinator.cpp
  src/action/action_planner.cpp
  src/requester/recognition.cpp
  src/requester/error_management.cpp
  src/requester/text_interpreter.cpp
  src/handler/system.cpp
  src/handler/movement.cpp
)

add_executable(node_brain
  src/node_brain.cpp
  ${SOURCES}
)

ament_target_dependencies(node_brain rclcpp std_msgs geometry_msgs std_srvs ament_index_cpp hexapod_interfaces nlohmann_json)

include_directories(include/hexapod_brain/)

install(TARGETS node_brain DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
