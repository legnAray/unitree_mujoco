cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

option(ENABLE_ROS2 "Enable ROS2 support for raycaster, odometry, and grid map publishers" OFF)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

if(ENABLE_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common)
  find_package(pcl_conversions REQUIRED)

  message(STATUS "ROS2 support enabled (raycaster + odom + gridmap)")
  add_definitions(-DENABLE_ROS2)
endif()

include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
  src
  raycaster/src
  raycaster/plugin
)
link_directories(mujoco/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

file(GLOB RAYCASTER_SRC raycaster/src/*.cpp raycaster/src/*.h raycaster/src/*.hpp)
file(GLOB RAYCASTER_PLUGIN_SRC raycaster/plugin/*.cc raycaster/plugin/*.h)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_library(sensor_ray SHARED ${RAYCASTER_PLUGIN_SRC} ${RAYCASTER_SRC})
target_include_directories(sensor_ray PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  raycaster/src
  raycaster/plugin
  mujoco/include
)
target_link_libraries(sensor_ray PRIVATE mujoco Eigen3::Eigen)

add_custom_command(TARGET sensor_ray POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/mujoco_plugin"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "$<TARGET_FILE:sensor_ray>"
    "${CMAKE_BINARY_DIR}/mujoco_plugin/"
  COMMENT "Copying sensor_ray plugin to mujoco_plugin directory..."
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco sensor_ray)

if(ENABLE_ROS2)
  find_package(grid_map_msgs REQUIRED)
  find_package(grid_map_ros REQUIRED)
  find_package(grid_map_cv REQUIRED)
  find_package(grid_map_core REQUIRED)
  find_package(CURL REQUIRED)

  remove_definitions(-DEIGEN_DENSEBASE_PLUGIN="grid_map_core/eigen_plugins/DenseBasePlugin.hpp")
  remove_definitions(-DEIGEN_FUNCTORS_PLUGIN="grid_map_core/eigen_plugins/FunctorsPlugin.hpp")

  target_compile_definitions(unitree_mujoco PRIVATE
    EIGEN_DENSEBASE_PLUGIN="grid_map_core/eigen_plugins/DenseBasePlugin.hpp"
    EIGEN_FUNCTORS_PLUGIN="grid_map_core/eigen_plugins/FunctorsPlugin.hpp"
  )

  target_include_directories(unitree_mujoco PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${grid_map_core_INCLUDE_DIRS}
    ${grid_map_ros_INCLUDE_DIRS}
    ${grid_map_cv_INCLUDE_DIRS}
  )
  target_link_libraries(unitree_mujoco
    ${PCL_LIBRARIES}
    CURL::libcurl
  )
  ament_target_dependencies(unitree_mujoco
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    grid_map_msgs
    grid_map_ros
    grid_map_cv
    grid_map_core
    pcl_conversions
  )
endif()

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
