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
CMAKE_SOURCE_DIR = /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen

# Include any dependencies generated for this target.
include CMakeFiles/mpc_codegen.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mpc_codegen.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mpc_codegen.dir/flags.make

CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.o: CMakeFiles/mpc_codegen.dir/flags.make
CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.o: mpc_codegen.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.o -c /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen/mpc_codegen.cpp

CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen/mpc_codegen.cpp > CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.i

CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen/mpc_codegen.cpp -o CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.s

# Object files for target mpc_codegen
mpc_codegen_OBJECTS = \
"CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.o"

# External object files for target mpc_codegen
mpc_codegen_EXTERNAL_OBJECTS =

mpc_codegen: CMakeFiles/mpc_codegen.dir/mpc_codegen.cpp.o
mpc_codegen: CMakeFiles/mpc_codegen.dir/build.make
mpc_codegen: /home/lty/ACADOtoolkit/build/lib/libacado_toolkit_s.so
mpc_codegen: CMakeFiles/mpc_codegen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mpc_codegen"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpc_codegen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mpc_codegen.dir/build: mpc_codegen

.PHONY : CMakeFiles/mpc_codegen.dir/build

CMakeFiles/mpc_codegen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mpc_codegen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mpc_codegen.dir/clean

CMakeFiles/mpc_codegen.dir/depend:
	cd /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen /home/lty/work_7.22/ustc/catkin_ws/src/cnuav_ros/cnuav_control/mpc_codegen/CMakeFiles/mpc_codegen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mpc_codegen.dir/depend

