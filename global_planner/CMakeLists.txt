# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.5)

# Set the project name
project (global_planner)


# https://github.com/lava/matplotlib-cpp/issues/236
# find python libraries
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})
# populate matplotplusplus repository
include(FetchContent)
FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master) # or whatever tag you want
FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

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

# add_executable(main ${SOURCES} main.cpp)
add_executable(main ${SOURCES} sim_astar.cpp)

# Set the directories that should be included in the build command for this target
# when running g++ these will be included as -I/directory/path/
target_include_directories(main
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(main PUBLIC matplot)