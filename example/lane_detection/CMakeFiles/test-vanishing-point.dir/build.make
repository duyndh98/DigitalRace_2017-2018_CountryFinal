# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/ubuntu/DriverlessCarChallenge_2017-2018/example

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/DriverlessCarChallenge_2017-2018/example

# Include any dependencies generated for this target.
include lane_detection/CMakeFiles/test-vanishing-point.dir/depend.make

# Include the progress variables for this target.
include lane_detection/CMakeFiles/test-vanishing-point.dir/progress.make

# Include the compile flags for this target's objects.
include lane_detection/CMakeFiles/test-vanishing-point.dir/flags.make

lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o: lane_detection/CMakeFiles/test-vanishing-point.dir/flags.make
lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o: lane_detection/test_lane_detection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/DriverlessCarChallenge_2017-2018/example/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o"
	cd /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o -c /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection/test_lane_detection.cpp

lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.i"
	cd /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection/test_lane_detection.cpp > CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.i

lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.s"
	cd /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection/test_lane_detection.cpp -o CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.s

lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o.requires:
.PHONY : lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o.requires

lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o.provides: lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o.requires
	$(MAKE) -f lane_detection/CMakeFiles/test-vanishing-point.dir/build.make lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o.provides.build
.PHONY : lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o.provides

lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o.provides.build: lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o

# Object files for target test-vanishing-point
test__vanishing__point_OBJECTS = \
"CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o"

# External object files for target test-vanishing-point
test__vanishing__point_EXTERNAL_OBJECTS =

bin/Release/test-vanishing-point: lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o
bin/Release/test-vanishing-point: lane_detection/CMakeFiles/test-vanishing-point.dir/build.make
bin/Release/test-vanishing-point: bin/Release/libvanishing-point.a
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_videostab.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_ccalib.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_fuzzy.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_rgbd.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_xfeatures2d.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_ximgproc.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_xobjdetect.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_xphoto.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_shape.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_video.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_calib3d.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_features2d.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_flann.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_objdetect.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_ml.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_highgui.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_photo.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_videoio.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_imgproc.so.3.2.0
bin/Release/test-vanishing-point: /usr/local/lib/libopencv_core.so.3.2.0
bin/Release/test-vanishing-point: lane_detection/CMakeFiles/test-vanishing-point.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/Release/test-vanishing-point"
	cd /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test-vanishing-point.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lane_detection/CMakeFiles/test-vanishing-point.dir/build: bin/Release/test-vanishing-point
.PHONY : lane_detection/CMakeFiles/test-vanishing-point.dir/build

lane_detection/CMakeFiles/test-vanishing-point.dir/requires: lane_detection/CMakeFiles/test-vanishing-point.dir/test_lane_detection.cpp.o.requires
.PHONY : lane_detection/CMakeFiles/test-vanishing-point.dir/requires

lane_detection/CMakeFiles/test-vanishing-point.dir/clean:
	cd /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection && $(CMAKE_COMMAND) -P CMakeFiles/test-vanishing-point.dir/cmake_clean.cmake
.PHONY : lane_detection/CMakeFiles/test-vanishing-point.dir/clean

lane_detection/CMakeFiles/test-vanishing-point.dir/depend:
	cd /home/ubuntu/DriverlessCarChallenge_2017-2018/example && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/DriverlessCarChallenge_2017-2018/example /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection /home/ubuntu/DriverlessCarChallenge_2017-2018/example /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection /home/ubuntu/DriverlessCarChallenge_2017-2018/example/lane_detection/CMakeFiles/test-vanishing-point.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lane_detection/CMakeFiles/test-vanishing-point.dir/depend

