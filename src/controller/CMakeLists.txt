cmake_minimum_required(VERSION 3.5)
project(controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  rclcpp
  nav_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
  Eigen3
  )
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Build the library
add_library(geometric_control
  src/controllers/geometric_controller.cpp
  src/controllers/pure_pursuit_controller.cpp
  src/controllers/model_acceleration_pursuit_controller.cpp
)
# Link library to dependencies
target_link_libraries (geometric_control
  Eigen3::Eigen
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(geometric_controller_node 
  src/geometric_controller_node.cpp
)
ament_target_dependencies(geometric_controller_node 
  rclcpp
  nav_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
)
target_link_libraries (geometric_controller_node 
  geometric_control
  Eigen3::Eigen
)


# Install Cpp executables
install(TARGETS
geometric_controller_node 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
