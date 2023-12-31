# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/Astar.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/Astar.cpp.o: ../src/Astar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/src/Astar.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/Astar.cpp.o -c /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/src/Astar.cpp

CMakeFiles/main.dir/src/Astar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/Astar.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/src/Astar.cpp > CMakeFiles/main.dir/src/Astar.cpp.i

CMakeFiles/main.dir/src/Astar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/Astar.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/src/Astar.cpp -o CMakeFiles/main.dir/src/Astar.cpp.s

CMakeFiles/main.dir/src/GridMap.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/GridMap.cpp.o: ../src/GridMap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/src/GridMap.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/GridMap.cpp.o -c /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/src/GridMap.cpp

CMakeFiles/main.dir/src/GridMap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/GridMap.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/src/GridMap.cpp > CMakeFiles/main.dir/src/GridMap.cpp.i

CMakeFiles/main.dir/src/GridMap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/GridMap.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/src/GridMap.cpp -o CMakeFiles/main.dir/src/GridMap.cpp.s

CMakeFiles/main.dir/src/PositionVector.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/PositionVector.cpp.o: ../src/PositionVector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/src/PositionVector.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/PositionVector.cpp.o -c /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/src/PositionVector.cpp

CMakeFiles/main.dir/src/PositionVector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/PositionVector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/src/PositionVector.cpp > CMakeFiles/main.dir/src/PositionVector.cpp.i

CMakeFiles/main.dir/src/PositionVector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/PositionVector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/src/PositionVector.cpp -o CMakeFiles/main.dir/src/PositionVector.cpp.s

CMakeFiles/main.dir/sim_astar.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/sim_astar.cpp.o: ../sim_astar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/main.dir/sim_astar.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/sim_astar.cpp.o -c /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/sim_astar.cpp

CMakeFiles/main.dir/sim_astar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/sim_astar.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/sim_astar.cpp > CMakeFiles/main.dir/sim_astar.cpp.i

CMakeFiles/main.dir/sim_astar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/sim_astar.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/sim_astar.cpp -o CMakeFiles/main.dir/sim_astar.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/Astar.cpp.o" \
"CMakeFiles/main.dir/src/GridMap.cpp.o" \
"CMakeFiles/main.dir/src/PositionVector.cpp.o" \
"CMakeFiles/main.dir/sim_astar.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/src/Astar.cpp.o
main: CMakeFiles/main.dir/src/GridMap.cpp.o
main: CMakeFiles/main.dir/src/PositionVector.cpp.o
main: CMakeFiles/main.dir/sim_astar.cpp.o
main: CMakeFiles/main.dir/build.make
main: _deps/matplotplusplus-build/source/matplot/libmatplot.a
main: /usr/lib/x86_64-linux-gnu/libjpeg.so
main: /usr/lib/x86_64-linux-gnu/libtiff.so
main: /usr/lib/x86_64-linux-gnu/libz.so
main: /usr/lib/x86_64-linux-gnu/libpng.so
main: /usr/lib/x86_64-linux-gnu/libz.so
main: /usr/lib/x86_64-linux-gnu/libpng.so
main: /usr/lib/x86_64-linux-gnu/liblapack.so
main: /usr/lib/x86_64-linux-gnu/libblas.so
main: _deps/matplotplusplus-build/source/3rd_party/libnodesoup.a
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build /home/justin/coding_projects/dstar_projects/trajectory_planning/global_planner/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

