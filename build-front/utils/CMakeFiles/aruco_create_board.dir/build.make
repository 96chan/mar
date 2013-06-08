# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ischool/Documents/aruco

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ischool/Documents/aruco/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/aruco_create_board.dir/depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/aruco_create_board.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/aruco_create_board.dir/flags.make

utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o: utils/CMakeFiles/aruco_create_board.dir/flags.make
utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o: ../utils/aruco_create_board.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ischool/Documents/aruco/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o"
	cd /home/ischool/Documents/aruco/build/utils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o -c /home/ischool/Documents/aruco/utils/aruco_create_board.cpp

utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.i"
	cd /home/ischool/Documents/aruco/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ischool/Documents/aruco/utils/aruco_create_board.cpp > CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.i

utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.s"
	cd /home/ischool/Documents/aruco/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ischool/Documents/aruco/utils/aruco_create_board.cpp -o CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.s

utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o.requires:
.PHONY : utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o.requires

utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o.provides: utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o.requires
	$(MAKE) -f utils/CMakeFiles/aruco_create_board.dir/build.make utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o.provides.build
.PHONY : utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o.provides

utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o.provides.build: utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o

# Object files for target aruco_create_board
aruco_create_board_OBJECTS = \
"CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o"

# External object files for target aruco_create_board
aruco_create_board_EXTERNAL_OBJECTS =

utils/aruco_create_board: utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o
utils/aruco_create_board: src/libaruco.so.1.2.4
utils/aruco_create_board: /usr/local/lib/libopencv_calib3d.so
utils/aruco_create_board: /usr/local/lib/libopencv_contrib.so
utils/aruco_create_board: /usr/local/lib/libopencv_core.so
utils/aruco_create_board: /usr/local/lib/libopencv_features2d.so
utils/aruco_create_board: /usr/local/lib/libopencv_flann.so
utils/aruco_create_board: /usr/local/lib/libopencv_gpu.so
utils/aruco_create_board: /usr/local/lib/libopencv_highgui.so
utils/aruco_create_board: /usr/local/lib/libopencv_imgproc.so
utils/aruco_create_board: /usr/local/lib/libopencv_legacy.so
utils/aruco_create_board: /usr/local/lib/libopencv_ml.so
utils/aruco_create_board: /usr/local/lib/libopencv_nonfree.so
utils/aruco_create_board: /usr/local/lib/libopencv_objdetect.so
utils/aruco_create_board: /usr/local/lib/libopencv_photo.so
utils/aruco_create_board: /usr/local/lib/libopencv_stitching.so
utils/aruco_create_board: /usr/local/lib/libopencv_superres.so
utils/aruco_create_board: /usr/local/lib/libopencv_ts.so
utils/aruco_create_board: /usr/local/lib/libopencv_video.so
utils/aruco_create_board: /usr/local/lib/libopencv_videostab.so
utils/aruco_create_board: utils/CMakeFiles/aruco_create_board.dir/build.make
utils/aruco_create_board: utils/CMakeFiles/aruco_create_board.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable aruco_create_board"
	cd /home/ischool/Documents/aruco/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_create_board.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/aruco_create_board.dir/build: utils/aruco_create_board
.PHONY : utils/CMakeFiles/aruco_create_board.dir/build

utils/CMakeFiles/aruco_create_board.dir/requires: utils/CMakeFiles/aruco_create_board.dir/aruco_create_board.cpp.o.requires
.PHONY : utils/CMakeFiles/aruco_create_board.dir/requires

utils/CMakeFiles/aruco_create_board.dir/clean:
	cd /home/ischool/Documents/aruco/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/aruco_create_board.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/aruco_create_board.dir/clean

utils/CMakeFiles/aruco_create_board.dir/depend:
	cd /home/ischool/Documents/aruco/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ischool/Documents/aruco /home/ischool/Documents/aruco/utils /home/ischool/Documents/aruco/build /home/ischool/Documents/aruco/build/utils /home/ischool/Documents/aruco/build/utils/CMakeFiles/aruco_create_board.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/aruco_create_board.dir/depend

