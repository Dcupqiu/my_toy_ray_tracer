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
CMAKE_SOURCE_DIR = /home/ubuntu/SourceCode/toy_ray_tracer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/SourceCode/toy_ray_tracer/build

# Include any dependencies generated for this target.
include CMakeFiles/theNextWeek.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/theNextWeek.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/theNextWeek.dir/flags.make

CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o: CMakeFiles/theNextWeek.dir/flags.make
CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o: ../src/TheNextWeek/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/SourceCode/toy_ray_tracer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o -c /home/ubuntu/SourceCode/toy_ray_tracer/src/TheNextWeek/main.cc

CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/SourceCode/toy_ray_tracer/src/TheNextWeek/main.cc > CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.i

CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/SourceCode/toy_ray_tracer/src/TheNextWeek/main.cc -o CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.s

CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o.requires:

.PHONY : CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o.requires

CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o.provides: CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o.requires
	$(MAKE) -f CMakeFiles/theNextWeek.dir/build.make CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o.provides.build
.PHONY : CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o.provides

CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o.provides.build: CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o


# Object files for target theNextWeek
theNextWeek_OBJECTS = \
"CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o"

# External object files for target theNextWeek
theNextWeek_EXTERNAL_OBJECTS =

theNextWeek: CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o
theNextWeek: CMakeFiles/theNextWeek.dir/build.make
theNextWeek: /usr/local/lib/libopencv_gapi.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_highgui.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_ml.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_objdetect.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_photo.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_stitching.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_video.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_videoio.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_dnn.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_imgcodecs.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_calib3d.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_features2d.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_flann.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_imgproc.so.4.5.1
theNextWeek: /usr/local/lib/libopencv_core.so.4.5.1
theNextWeek: CMakeFiles/theNextWeek.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/SourceCode/toy_ray_tracer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable theNextWeek"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/theNextWeek.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/theNextWeek.dir/build: theNextWeek

.PHONY : CMakeFiles/theNextWeek.dir/build

CMakeFiles/theNextWeek.dir/requires: CMakeFiles/theNextWeek.dir/src/TheNextWeek/main.cc.o.requires

.PHONY : CMakeFiles/theNextWeek.dir/requires

CMakeFiles/theNextWeek.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/theNextWeek.dir/cmake_clean.cmake
.PHONY : CMakeFiles/theNextWeek.dir/clean

CMakeFiles/theNextWeek.dir/depend:
	cd /home/ubuntu/SourceCode/toy_ray_tracer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/SourceCode/toy_ray_tracer /home/ubuntu/SourceCode/toy_ray_tracer /home/ubuntu/SourceCode/toy_ray_tracer/build /home/ubuntu/SourceCode/toy_ray_tracer/build /home/ubuntu/SourceCode/toy_ray_tracer/build/CMakeFiles/theNextWeek.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/theNextWeek.dir/depend

