# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/marcus/Workspace/Coding/C++/KalmanEigen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marcus/Workspace/Coding/C++/KalmanEigen/build

# Include any dependencies generated for this target.
include CMakeFiles/Kalman.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Kalman.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Kalman.dir/flags.make

CMakeFiles/Kalman.dir/src/Kalman.cpp.o: CMakeFiles/Kalman.dir/flags.make
CMakeFiles/Kalman.dir/src/Kalman.cpp.o: ../src/Kalman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/marcus/Workspace/Coding/C++/KalmanEigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Kalman.dir/src/Kalman.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Kalman.dir/src/Kalman.cpp.o -c /home/marcus/Workspace/Coding/C++/KalmanEigen/src/Kalman.cpp

CMakeFiles/Kalman.dir/src/Kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kalman.dir/src/Kalman.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/marcus/Workspace/Coding/C++/KalmanEigen/src/Kalman.cpp > CMakeFiles/Kalman.dir/src/Kalman.cpp.i

CMakeFiles/Kalman.dir/src/Kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kalman.dir/src/Kalman.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/marcus/Workspace/Coding/C++/KalmanEigen/src/Kalman.cpp -o CMakeFiles/Kalman.dir/src/Kalman.cpp.s

CMakeFiles/Kalman.dir/src/Kalman.cpp.o.requires:

.PHONY : CMakeFiles/Kalman.dir/src/Kalman.cpp.o.requires

CMakeFiles/Kalman.dir/src/Kalman.cpp.o.provides: CMakeFiles/Kalman.dir/src/Kalman.cpp.o.requires
	$(MAKE) -f CMakeFiles/Kalman.dir/build.make CMakeFiles/Kalman.dir/src/Kalman.cpp.o.provides.build
.PHONY : CMakeFiles/Kalman.dir/src/Kalman.cpp.o.provides

CMakeFiles/Kalman.dir/src/Kalman.cpp.o.provides.build: CMakeFiles/Kalman.dir/src/Kalman.cpp.o


CMakeFiles/Kalman.dir/src/main.cpp.o: CMakeFiles/Kalman.dir/flags.make
CMakeFiles/Kalman.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/marcus/Workspace/Coding/C++/KalmanEigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Kalman.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Kalman.dir/src/main.cpp.o -c /home/marcus/Workspace/Coding/C++/KalmanEigen/src/main.cpp

CMakeFiles/Kalman.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kalman.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/marcus/Workspace/Coding/C++/KalmanEigen/src/main.cpp > CMakeFiles/Kalman.dir/src/main.cpp.i

CMakeFiles/Kalman.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kalman.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/marcus/Workspace/Coding/C++/KalmanEigen/src/main.cpp -o CMakeFiles/Kalman.dir/src/main.cpp.s

CMakeFiles/Kalman.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/Kalman.dir/src/main.cpp.o.requires

CMakeFiles/Kalman.dir/src/main.cpp.o.provides: CMakeFiles/Kalman.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Kalman.dir/build.make CMakeFiles/Kalman.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/Kalman.dir/src/main.cpp.o.provides

CMakeFiles/Kalman.dir/src/main.cpp.o.provides.build: CMakeFiles/Kalman.dir/src/main.cpp.o


# Object files for target Kalman
Kalman_OBJECTS = \
"CMakeFiles/Kalman.dir/src/Kalman.cpp.o" \
"CMakeFiles/Kalman.dir/src/main.cpp.o"

# External object files for target Kalman
Kalman_EXTERNAL_OBJECTS =

bin/Kalman: CMakeFiles/Kalman.dir/src/Kalman.cpp.o
bin/Kalman: CMakeFiles/Kalman.dir/src/main.cpp.o
bin/Kalman: CMakeFiles/Kalman.dir/build.make
bin/Kalman: CMakeFiles/Kalman.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/marcus/Workspace/Coding/C++/KalmanEigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable bin/Kalman"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Kalman.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Kalman.dir/build: bin/Kalman

.PHONY : CMakeFiles/Kalman.dir/build

CMakeFiles/Kalman.dir/requires: CMakeFiles/Kalman.dir/src/Kalman.cpp.o.requires
CMakeFiles/Kalman.dir/requires: CMakeFiles/Kalman.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/Kalman.dir/requires

CMakeFiles/Kalman.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Kalman.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Kalman.dir/clean

CMakeFiles/Kalman.dir/depend:
	cd /home/marcus/Workspace/Coding/C++/KalmanEigen/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marcus/Workspace/Coding/C++/KalmanEigen /home/marcus/Workspace/Coding/C++/KalmanEigen /home/marcus/Workspace/Coding/C++/KalmanEigen/build /home/marcus/Workspace/Coding/C++/KalmanEigen/build /home/marcus/Workspace/Coding/C++/KalmanEigen/build/CMakeFiles/Kalman.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Kalman.dir/depend
