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
CMAKE_SOURCE_DIR = /home/yiming/training/train/summuer_training_1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yiming/training/train/summuer_training_1/build

# Include any dependencies generated for this target.
include CMakeFiles/MA.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MA.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MA.dir/flags.make

CMakeFiles/MA.dir/pnp_for_training.cpp.o: CMakeFiles/MA.dir/flags.make
CMakeFiles/MA.dir/pnp_for_training.cpp.o: ../pnp_for_training.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yiming/training/train/summuer_training_1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MA.dir/pnp_for_training.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MA.dir/pnp_for_training.cpp.o -c /home/yiming/training/train/summuer_training_1/pnp_for_training.cpp

CMakeFiles/MA.dir/pnp_for_training.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MA.dir/pnp_for_training.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yiming/training/train/summuer_training_1/pnp_for_training.cpp > CMakeFiles/MA.dir/pnp_for_training.cpp.i

CMakeFiles/MA.dir/pnp_for_training.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MA.dir/pnp_for_training.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yiming/training/train/summuer_training_1/pnp_for_training.cpp -o CMakeFiles/MA.dir/pnp_for_training.cpp.s

# Object files for target MA
MA_OBJECTS = \
"CMakeFiles/MA.dir/pnp_for_training.cpp.o"

# External object files for target MA
MA_EXTERNAL_OBJECTS =

bin/MA: CMakeFiles/MA.dir/pnp_for_training.cpp.o
bin/MA: CMakeFiles/MA.dir/build.make
bin/MA: modules/libmodules.so
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_gapi.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_highgui.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_ml.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_objdetect.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_photo.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_stitching.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_video.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_calib3d.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_dnn.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_features2d.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_flann.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_videoio.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_imgcodecs.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_imgproc.so.4.5.3
bin/MA: /opt/intel/openvino_2021/opencv/lib/libopencv_core.so.4.5.3
bin/MA: /usr/local/lib/libfmt.so.8.1.1
bin/MA: CMakeFiles/MA.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yiming/training/train/summuer_training_1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/MA"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MA.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MA.dir/build: bin/MA

.PHONY : CMakeFiles/MA.dir/build

CMakeFiles/MA.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MA.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MA.dir/clean

CMakeFiles/MA.dir/depend:
	cd /home/yiming/training/train/summuer_training_1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yiming/training/train/summuer_training_1 /home/yiming/training/train/summuer_training_1 /home/yiming/training/train/summuer_training_1/build /home/yiming/training/train/summuer_training_1/build /home/yiming/training/train/summuer_training_1/build/CMakeFiles/MA.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MA.dir/depend

