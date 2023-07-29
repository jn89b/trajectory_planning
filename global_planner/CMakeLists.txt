# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.5)

# Set the project name
project (global_planner)

#include Eigen
include_directories(/usr/include/eigen3)


# Create a sources variable with a link to all cpp files to compile
set(SOURCES
    src/Astar.cpp
    src/GridMap.cpp
    src/PositionVector.cpp
)

# Add an executable with the above sources
# add_executable(taylor_methods src/taylor_methods.cpp)
# add_executable(euler_methods src/euler_methods.cpp)

add_executable(main ${SOURCES} main.cpp)

# Set the directories that should be included in the build command for this target
# when running g++ these will be included as -I/directory/path/
target_include_directories(main
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)
