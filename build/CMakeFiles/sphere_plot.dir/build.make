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
include CMakeFiles/sphere_plot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sphere_plot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sphere_plot.dir/flags.make

CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o: CMakeFiles/sphere_plot.dir/flags.make
CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o: ../src/TheRestOfYourLife/sphere_plot.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/SourceCode/toy_ray_tracer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o -c /home/ubuntu/SourceCode/toy_ray_tracer/src/TheRestOfYourLife/sphere_plot.cc

CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/SourceCode/toy_ray_tracer/src/TheRestOfYourLife/sphere_plot.cc > CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.i

CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/SourceCode/toy_ray_tracer/src/TheRestOfYourLife/sphere_plot.cc -o CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.s

CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o.requires:

.PHONY : CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o.requires

CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o.provides: CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o.requires
	$(MAKE) -f CMakeFiles/sphere_plot.dir/build.make CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o.provides.build
.PHONY : CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o.provides

CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o.provides.build: CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o


# Object files for target sphere_plot
sphere_plot_OBJECTS = \
"CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o"

# External object files for target sphere_plot
sphere_plot_EXTERNAL_OBJECTS =

sphere_plot: CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o
sphere_plot: CMakeFiles/sphere_plot.dir/build.make
sphere_plot: CMakeFiles/sphere_plot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/SourceCode/toy_ray_tracer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sphere_plot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sphere_plot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sphere_plot.dir/build: sphere_plot

.PHONY : CMakeFiles/sphere_plot.dir/build

CMakeFiles/sphere_plot.dir/requires: CMakeFiles/sphere_plot.dir/src/TheRestOfYourLife/sphere_plot.cc.o.requires

.PHONY : CMakeFiles/sphere_plot.dir/requires

CMakeFiles/sphere_plot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sphere_plot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sphere_plot.dir/clean

CMakeFiles/sphere_plot.dir/depend:
	cd /home/ubuntu/SourceCode/toy_ray_tracer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/SourceCode/toy_ray_tracer /home/ubuntu/SourceCode/toy_ray_tracer /home/ubuntu/SourceCode/toy_ray_tracer/build /home/ubuntu/SourceCode/toy_ray_tracer/build /home/ubuntu/SourceCode/toy_ray_tracer/build/CMakeFiles/sphere_plot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sphere_plot.dir/depend

