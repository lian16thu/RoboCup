# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/lian/robot_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lian/robot_ws/build

# Include any dependencies generated for this target.
include head_motion/CMakeFiles/head_control.dir/depend.make

# Include the progress variables for this target.
include head_motion/CMakeFiles/head_control.dir/progress.make

# Include the compile flags for this target's objects.
include head_motion/CMakeFiles/head_control.dir/flags.make

head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o: head_motion/CMakeFiles/head_control.dir/flags.make
head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o: /home/lian/robot_ws/src/head_motion/src/head_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lian/robot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o"
	cd /home/lian/robot_ws/build/head_motion && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/head_control.dir/src/head_control.cpp.o -c /home/lian/robot_ws/src/head_motion/src/head_control.cpp

head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/head_control.dir/src/head_control.cpp.i"
	cd /home/lian/robot_ws/build/head_motion && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lian/robot_ws/src/head_motion/src/head_control.cpp > CMakeFiles/head_control.dir/src/head_control.cpp.i

head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/head_control.dir/src/head_control.cpp.s"
	cd /home/lian/robot_ws/build/head_motion && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lian/robot_ws/src/head_motion/src/head_control.cpp -o CMakeFiles/head_control.dir/src/head_control.cpp.s

head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o.requires:

.PHONY : head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o.requires

head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o.provides: head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o.requires
	$(MAKE) -f head_motion/CMakeFiles/head_control.dir/build.make head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o.provides.build
.PHONY : head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o.provides

head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o.provides.build: head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o


# Object files for target head_control
head_control_OBJECTS = \
"CMakeFiles/head_control.dir/src/head_control.cpp.o"

# External object files for target head_control
head_control_EXTERNAL_OBJECTS =

/home/lian/robot_ws/devel/lib/head_motion/head_control: head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o
/home/lian/robot_ws/devel/lib/head_motion/head_control: head_motion/CMakeFiles/head_control.dir/build.make
/home/lian/robot_ws/devel/lib/head_motion/head_control: /opt/ros/kinetic/lib/libroscpp.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /opt/ros/kinetic/lib/librosconsole.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /opt/ros/kinetic/lib/librostime.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /opt/ros/kinetic/lib/libcpp_common.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: /home/lian/robot_ws/devel/lib/libdxl_head_motor_driver.so
/home/lian/robot_ws/devel/lib/head_motion/head_control: head_motion/CMakeFiles/head_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lian/robot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/lian/robot_ws/devel/lib/head_motion/head_control"
	cd /home/lian/robot_ws/build/head_motion && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/head_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
head_motion/CMakeFiles/head_control.dir/build: /home/lian/robot_ws/devel/lib/head_motion/head_control

.PHONY : head_motion/CMakeFiles/head_control.dir/build

head_motion/CMakeFiles/head_control.dir/requires: head_motion/CMakeFiles/head_control.dir/src/head_control.cpp.o.requires

.PHONY : head_motion/CMakeFiles/head_control.dir/requires

head_motion/CMakeFiles/head_control.dir/clean:
	cd /home/lian/robot_ws/build/head_motion && $(CMAKE_COMMAND) -P CMakeFiles/head_control.dir/cmake_clean.cmake
.PHONY : head_motion/CMakeFiles/head_control.dir/clean

head_motion/CMakeFiles/head_control.dir/depend:
	cd /home/lian/robot_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lian/robot_ws/src /home/lian/robot_ws/src/head_motion /home/lian/robot_ws/build /home/lian/robot_ws/build/head_motion /home/lian/robot_ws/build/head_motion/CMakeFiles/head_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : head_motion/CMakeFiles/head_control.dir/depend
