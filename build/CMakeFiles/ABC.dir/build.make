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
CMAKE_SOURCE_DIR = /home/er/Documents/d_development/MMVII/TestER

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/er/Documents/d_development/MMVII/TestER/build

# Include any dependencies generated for this target.
include CMakeFiles/ABC.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ABC.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ABC.dir/flags.make

CMakeFiles/ABC.dir/src/main.cpp.o: CMakeFiles/ABC.dir/flags.make
CMakeFiles/ABC.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/er/Documents/d_development/MMVII/TestER/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ABC.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ABC.dir/src/main.cpp.o -c /home/er/Documents/d_development/MMVII/TestER/src/main.cpp

CMakeFiles/ABC.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ABC.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/er/Documents/d_development/MMVII/TestER/src/main.cpp > CMakeFiles/ABC.dir/src/main.cpp.i

CMakeFiles/ABC.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ABC.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/er/Documents/d_development/MMVII/TestER/src/main.cpp -o CMakeFiles/ABC.dir/src/main.cpp.s

CMakeFiles/ABC.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/ABC.dir/src/main.cpp.o.requires

CMakeFiles/ABC.dir/src/main.cpp.o.provides: CMakeFiles/ABC.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/ABC.dir/build.make CMakeFiles/ABC.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/ABC.dir/src/main.cpp.o.provides

CMakeFiles/ABC.dir/src/main.cpp.o.provides.build: CMakeFiles/ABC.dir/src/main.cpp.o


CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o: CMakeFiles/ABC.dir/flags.make
CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o: ../src/TestEqCollinear.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/er/Documents/d_development/MMVII/TestER/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o -c /home/er/Documents/d_development/MMVII/TestER/src/TestEqCollinear.cpp

CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/er/Documents/d_development/MMVII/TestER/src/TestEqCollinear.cpp > CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.i

CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/er/Documents/d_development/MMVII/TestER/src/TestEqCollinear.cpp -o CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.s

CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o.requires:

.PHONY : CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o.requires

CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o.provides: CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o.requires
	$(MAKE) -f CMakeFiles/ABC.dir/build.make CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o.provides.build
.PHONY : CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o.provides

CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o.provides.build: CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o


CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o: CMakeFiles/ABC.dir/flags.make
CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o: ../src/Graphs/GraphBasedBA.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/er/Documents/d_development/MMVII/TestER/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o -c /home/er/Documents/d_development/MMVII/TestER/src/Graphs/GraphBasedBA.cpp

CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/er/Documents/d_development/MMVII/TestER/src/Graphs/GraphBasedBA.cpp > CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.i

CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/er/Documents/d_development/MMVII/TestER/src/Graphs/GraphBasedBA.cpp -o CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.s

CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o.requires:

.PHONY : CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o.requires

CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o.provides: CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o.requires
	$(MAKE) -f CMakeFiles/ABC.dir/build.make CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o.provides.build
.PHONY : CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o.provides

CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o.provides.build: CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o


# Object files for target ABC
ABC_OBJECTS = \
"CMakeFiles/ABC.dir/src/main.cpp.o" \
"CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o" \
"CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o"

# External object files for target ABC
ABC_EXTERNAL_OBJECTS =

ABC: CMakeFiles/ABC.dir/src/main.cpp.o
ABC: CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o
ABC: CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o
ABC: CMakeFiles/ABC.dir/build.make
ABC: /usr/local/lib/libceres.a
ABC: /usr/lib/x86_64-linux-gnu/libglog.so
ABC: /usr/lib/x86_64-linux-gnu/libgflags.so
ABC: /usr/lib/x86_64-linux-gnu/libspqr.so
ABC: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
ABC: /usr/lib/x86_64-linux-gnu/libtbb.so
ABC: /usr/lib/x86_64-linux-gnu/libcholmod.so
ABC: /usr/lib/x86_64-linux-gnu/libccolamd.so
ABC: /usr/lib/x86_64-linux-gnu/libcamd.so
ABC: /usr/lib/x86_64-linux-gnu/libcolamd.so
ABC: /usr/lib/x86_64-linux-gnu/libamd.so
ABC: /usr/lib/liblapack.so
ABC: /usr/lib/libf77blas.so
ABC: /usr/lib/libatlas.so
ABC: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
ABC: /usr/lib/x86_64-linux-gnu/librt.so
ABC: /usr/lib/x86_64-linux-gnu/libcxsparse.so
ABC: /usr/lib/liblapack.so
ABC: /usr/lib/libf77blas.so
ABC: /usr/lib/libatlas.so
ABC: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
ABC: /usr/lib/x86_64-linux-gnu/librt.so
ABC: /usr/lib/x86_64-linux-gnu/libcxsparse.so
ABC: CMakeFiles/ABC.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/er/Documents/d_development/MMVII/TestER/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ABC"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ABC.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ABC.dir/build: ABC

.PHONY : CMakeFiles/ABC.dir/build

CMakeFiles/ABC.dir/requires: CMakeFiles/ABC.dir/src/main.cpp.o.requires
CMakeFiles/ABC.dir/requires: CMakeFiles/ABC.dir/src/TestEqCollinear.cpp.o.requires
CMakeFiles/ABC.dir/requires: CMakeFiles/ABC.dir/src/Graphs/GraphBasedBA.cpp.o.requires

.PHONY : CMakeFiles/ABC.dir/requires

CMakeFiles/ABC.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ABC.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ABC.dir/clean

CMakeFiles/ABC.dir/depend:
	cd /home/er/Documents/d_development/MMVII/TestER/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/er/Documents/d_development/MMVII/TestER /home/er/Documents/d_development/MMVII/TestER /home/er/Documents/d_development/MMVII/TestER/build /home/er/Documents/d_development/MMVII/TestER/build /home/er/Documents/d_development/MMVII/TestER/build/CMakeFiles/ABC.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ABC.dir/depend

