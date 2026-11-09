cmake_minimum_required(VERSION 3.16)
project(emsd_physics LANGUAGES CXX)
find_package(Python3 REQUIRED COMPONENTS Development)

# Prefer Gazebo Harmonic (ROS 2 Jazzy, Ubuntu 24.04); fall back to Gazebo
# Fortress (ROS 2 Humble, Ubuntu 22.04). Force Fortress with -DEMSD_GZ=fortress.
set(EMSD_GZ "auto" CACHE STRING "Gazebo version: auto, harmonic or fortress")
if(NOT EMSD_GZ STREQUAL "fortress")
  find_package(gz-sim8 QUIET)
endif()
if(gz-sim8_FOUND)
  set(EMSD_GZ_HARMONIC 1)
  find_package(gz-plugin2 REQUIRED COMPONENTS register loader)
  find_package(gz-physics7 REQUIRED COMPONENTS sdf)
  set(EMSD_SIM_TARGET gz-sim8::core)
  set(EMSD_PLUGIN_REGISTER gz-plugin2::register)
  set(EMSD_PLUGIN_LOADER gz-plugin2::loader)
  set(EMSD_PHYSICS_SDF gz-physics7::sdf)
  set(EMSD_DART_NAMES gz-physics7-dartsim-plugin gz-physics-dartsim-plugin)
  # The DART engine library lives beside gz-physics7's own libraries.
  get_target_property(EMSD_PHYSICS_LIB gz-physics7::core LOCATION)
  get_filename_component(EMSD_PHYSICS_LIBDIR "${EMSD_PHYSICS_LIB}" DIRECTORY)
  set(EMSD_DART_HINTS "${EMSD_PHYSICS_LIBDIR}")
  set(EMSD_DART_SUFFIXES gz-physics-7/engine-plugins)
  message(STATUS "EMSD: building against Gazebo Harmonic (gz-sim8)")
elseif(NOT EMSD_GZ STREQUAL "harmonic")
  set(EMSD_GZ_HARMONIC 0)
  find_package(ignition-gazebo6 REQUIRED)
  find_package(ignition-plugin1 REQUIRED COMPONENTS register loader)
  find_package(ignition-physics5 REQUIRED COMPONENTS sdf)
  set(EMSD_SIM_TARGET ignition-gazebo6::core)
  set(EMSD_PLUGIN_REGISTER ignition-plugin1::register)
  set(EMSD_PLUGIN_LOADER ignition-plugin1::loader)
  set(EMSD_PHYSICS_SDF ignition-physics5::sdf)
  set(EMSD_DART_NAMES ignition-physics5-dartsim-plugin)
  set(EMSD_DART_HINTS "")
  set(EMSD_DART_SUFFIXES ign-physics-5/engine-plugins)
  message(STATUS "EMSD: building against Gazebo Fortress (ignition-gazebo6)")
else()
  message(FATAL_ERROR "EMSD_GZ=harmonic but gz-sim8 was not found")
endif()

find_library(EMSD_DART_ENGINE NAMES ${EMSD_DART_NAMES}
  HINTS ${EMSD_DART_HINTS} PATH_SUFFIXES ${EMSD_DART_SUFFIXES})
if(NOT EMSD_DART_ENGINE)
  message(FATAL_ERROR "Gazebo DART physics engine is required")
endif()

add_library(emsd_motor_physics SHARED MotorPhysics.cc)
target_compile_features(emsd_motor_physics PRIVATE cxx_std_17)
target_compile_options(emsd_motor_physics PRIVATE -Wall -Wextra)
target_compile_definitions(emsd_motor_physics PRIVATE EMSD_GZ_HARMONIC=${EMSD_GZ_HARMONIC})
target_link_libraries(emsd_motor_physics PRIVATE
  ${EMSD_SIM_TARGET} ${EMSD_PLUGIN_REGISTER} Python3::Python ${CMAKE_DL_LIBS})
add_executable(emsd_headless headless.cc)
target_compile_features(emsd_headless PRIVATE cxx_std_17)
target_compile_definitions(emsd_headless PRIVATE
  EMSD_DART_ENGINE="${EMSD_DART_ENGINE}" EMSD_GZ_HARMONIC=${EMSD_GZ_HARMONIC})
target_link_libraries(emsd_headless PRIVATE ${EMSD_SIM_TARGET}
  ${EMSD_PLUGIN_REGISTER} ${EMSD_PLUGIN_LOADER} ${EMSD_PHYSICS_SDF} Python3::Python
  ${CMAKE_DL_LIBS})
include(CTest)
if(BUILD_TESTING)
  add_executable(test_motor_physics test_motor_physics.cc)
  target_compile_features(test_motor_physics PRIVATE cxx_std_17)
  target_compile_definitions(test_motor_physics PRIVATE EMSD_GZ_HARMONIC=${EMSD_GZ_HARMONIC})
  target_link_libraries(test_motor_physics PRIVATE
    ${EMSD_SIM_TARGET} ${EMSD_PLUGIN_REGISTER} Python3::Python ${CMAKE_DL_LIBS})
  add_test(NAME motor_physics_bridge COMMAND test_motor_physics
    ${CMAKE_CURRENT_SOURCE_DIR}/../control
    ${CMAKE_CURRENT_SOURCE_DIR}/../control/config/drone.json)
endif()
