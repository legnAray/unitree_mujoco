cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

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

if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(UNITREE_LEGGED_SDK_ARCH "amd64")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(UNITREE_LEGGED_SDK_ARCH "arm64")
else()
  message(WARNING "Unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}' for unitree_legged_sdk.")
endif()

set(UNITREE_LEGGED_SDK_HOME "${CMAKE_SOURCE_DIR}/../../stepit/extern/robot_sdk/unitree_aliengo_sdk")
if (EXISTS "${UNITREE_LEGGED_SDK_HOME}/include")
  include_directories(${UNITREE_LEGGED_SDK_HOME}/include)
  if (UNITREE_LEGGED_SDK_ARCH)
    link_directories(${UNITREE_LEGGED_SDK_HOME}/lib/cpp/${UNITREE_LEGGED_SDK_ARCH})
    list(APPEND CMAKE_BUILD_RPATH "${UNITREE_LEGGED_SDK_HOME}/lib/cpp/${UNITREE_LEGGED_SDK_ARCH}")
  endif()
endif()


include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
)
link_directories(mujoco/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc
  src/unitree_legged_bridge.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  unitree_legged_sdk
  boost_program_options
  fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
