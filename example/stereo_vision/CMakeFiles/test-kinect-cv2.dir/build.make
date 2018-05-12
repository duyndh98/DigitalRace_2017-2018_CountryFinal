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
CMAKE_SOURCE_DIR = /home/ubuntu/FinalRound-CDS/example

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/FinalRound-CDS/example

# Include any dependencies generated for this target.
include stereo_vision/CMakeFiles/test-kinect-cv2.dir/depend.make

# Include the progress variables for this target.
include stereo_vision/CMakeFiles/test-kinect-cv2.dir/progress.make

# Include the compile flags for this target's objects.
include stereo_vision/CMakeFiles/test-kinect-cv2.dir/flags.make

stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o: stereo_vision/CMakeFiles/test-kinect-cv2.dir/flags.make
stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o: stereo_vision/test_kinect_cv2.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/FinalRound-CDS/example/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o"
	cd /home/ubuntu/FinalRound-CDS/example/stereo_vision && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o -c /home/ubuntu/FinalRound-CDS/example/stereo_vision/test_kinect_cv2.cpp

stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.i"
	cd /home/ubuntu/FinalRound-CDS/example/stereo_vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/FinalRound-CDS/example/stereo_vision/test_kinect_cv2.cpp > CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.i

stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.s"
	cd /home/ubuntu/FinalRound-CDS/example/stereo_vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/FinalRound-CDS/example/stereo_vision/test_kinect_cv2.cpp -o CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.s

stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o.requires:
.PHONY : stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o.requires

stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o.provides: stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o.requires
	$(MAKE) -f stereo_vision/CMakeFiles/test-kinect-cv2.dir/build.make stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o.provides.build
.PHONY : stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o.provides

stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o.provides.build: stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o

# Object files for target test-kinect-cv2
test__kinect__cv2_OBJECTS = \
"CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o"

# External object files for target test-kinect-cv2
test__kinect__cv2_EXTERNAL_OBJECTS =

bin/Release/test-kinect-cv2: stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o
bin/Release/test-kinect-cv2: stereo_vision/CMakeFiles/test-kinect-cv2.dir/build.make
bin/Release/test-kinect-cv2: bin/Release/libkinect-cv2.a
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_videostab.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_ccalib.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_fuzzy.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_rgbd.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_xfeatures2d.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_shape.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_video.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_ximgproc.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_calib3d.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_features2d.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_flann.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_xobjdetect.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_objdetect.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_ml.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_xphoto.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_highgui.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_photo.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_videoio.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_imgproc.so.3.2.0
bin/Release/test-kinect-cv2: /usr/local/lib/libopencv_core.so.3.2.0
bin/Release/test-kinect-cv2: stereo_vision/CMakeFiles/test-kinect-cv2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/Release/test-kinect-cv2"
	cd /home/ubuntu/FinalRound-CDS/example/stereo_vision && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test-kinect-cv2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
stereo_vision/CMakeFiles/test-kinect-cv2.dir/build: bin/Release/test-kinect-cv2
.PHONY : stereo_vision/CMakeFiles/test-kinect-cv2.dir/build

stereo_vision/CMakeFiles/test-kinect-cv2.dir/requires: stereo_vision/CMakeFiles/test-kinect-cv2.dir/test_kinect_cv2.cpp.o.requires
.PHONY : stereo_vision/CMakeFiles/test-kinect-cv2.dir/requires

stereo_vision/CMakeFiles/test-kinect-cv2.dir/clean:
	cd /home/ubuntu/FinalRound-CDS/example/stereo_vision && $(CMAKE_COMMAND) -P CMakeFiles/test-kinect-cv2.dir/cmake_clean.cmake
.PHONY : stereo_vision/CMakeFiles/test-kinect-cv2.dir/clean

stereo_vision/CMakeFiles/test-kinect-cv2.dir/depend:
	cd /home/ubuntu/FinalRound-CDS/example && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/FinalRound-CDS/example /home/ubuntu/FinalRound-CDS/example/stereo_vision /home/ubuntu/FinalRound-CDS/example /home/ubuntu/FinalRound-CDS/example/stereo_vision /home/ubuntu/FinalRound-CDS/example/stereo_vision/CMakeFiles/test-kinect-cv2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : stereo_vision/CMakeFiles/test-kinect-cv2.dir/depend

