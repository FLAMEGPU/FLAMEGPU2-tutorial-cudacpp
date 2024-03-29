cmake_minimum_required(VERSION VERSION 3.18 FATAL_ERROR)

# Name the project and set languages
project(flamegpu2_tutorial CUDA CXX)

# Option to enable/disable building the static library
option(VISUALISATION "Enable visualisation support" OFF)

# Optionaly set the version of flamegpu which should be used.
# Use a branch name such as `master`, or a tagged version such as `v2.0.0-alpha` 
set(FLAMEGPU_VERSION "v2.0.0-alpha.1" CACHE STRING "Git branch or tag to use")

# Our core dependency is FLAMEGPU2 lib, first lets find it
include(${CMAKE_CURRENT_LIST_DIR}/cmake/flamegpu2.cmake)

# Include common rules from the main repository's CMake
include(${FLAMEGPU_ROOT}/cmake/common.cmake)

# Define output location of binary files
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}/)

# Prepare list of source files
# Can't do this automatically, as CMake wouldn't know when to regen (as CMakeLists.txt would be unchanged)
SET(ALL_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cu
)

# Add the executable and set required flags for the target
add_flamegpu_executable("${PROJECT_NAME}" "${ALL_SRC}" "${FLAMEGPU_ROOT}" "${PROJECT_BINARY_DIR}" TRUE)

# Add src directory to include path
target_include_directories("${PROJECT_NAME}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Also set as startup project
set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"  PROPERTY VS_STARTUP_PROJECT "${PROJECT_NAME}")

# Set the default (visual studio) debugger configure_file
set_target_properties("${PROJECT_NAME}" PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                                                   VS_DEBUGGER_COMMAND_ARGUMENTS "-i iterations/0.xml -s 1")