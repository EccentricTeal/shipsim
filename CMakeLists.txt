cmake_minimum_required(VERSION 3.5)
project(shipsim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package( ament_cmake REQUIRED )
find_package( rclcpp REQUIRED )
find_package( tf2_ros REQUIRED )
find_package( std_msgs REQUIRED )
find_package( geometry_msgs REQUIRED )
find_package( shipsim_model REQUIRED )
find_package( shipsim_msgs REQUIRED )

#Build
add_executable(shipsim_node
  src/shipsim/node.cc
  src/shipsim/core.cc
)
ament_target_dependencies(shipsim_node
  rclcpp
  tf2_ros
  std_msgs
  geometry_msgs
  shipsim_model
  shipsim_msgs
)
target_include_directories(shipsim_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


#Install
install(TARGETS
  shipsim_node
  DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

ament_package()
