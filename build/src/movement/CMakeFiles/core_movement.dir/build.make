# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/elpistars/elpistars/Project_Humanoid

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/elpistars/elpistars/Project_Humanoid/build

# Include any dependencies generated for this target.
include src/movement/CMakeFiles/core_movement.dir/depend.make

# Include the progress variables for this target.
include src/movement/CMakeFiles/core_movement.dir/progress.make

# Include the compile flags for this target's objects.
include src/movement/CMakeFiles/core_movement.dir/flags.make

src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o: src/movement/CMakeFiles/core_movement.dir/flags.make
src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o: ../src/movement/foot_ik.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/elpistars/elpistars/Project_Humanoid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core_movement.dir/foot_ik.cpp.o -c /home/elpistars/elpistars/Project_Humanoid/src/movement/foot_ik.cpp

src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core_movement.dir/foot_ik.cpp.i"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/elpistars/elpistars/Project_Humanoid/src/movement/foot_ik.cpp > CMakeFiles/core_movement.dir/foot_ik.cpp.i

src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core_movement.dir/foot_ik.cpp.s"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/elpistars/elpistars/Project_Humanoid/src/movement/foot_ik.cpp -o CMakeFiles/core_movement.dir/foot_ik.cpp.s

src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o.requires:

.PHONY : src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o.requires

src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o.provides: src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o.requires
	$(MAKE) -f src/movement/CMakeFiles/core_movement.dir/build.make src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o.provides.build
.PHONY : src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o.provides

src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o.provides.build: src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o


src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o: src/movement/CMakeFiles/core_movement.dir/flags.make
src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o: ../src/movement/berdiri.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/elpistars/elpistars/Project_Humanoid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core_movement.dir/berdiri.cpp.o -c /home/elpistars/elpistars/Project_Humanoid/src/movement/berdiri.cpp

src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core_movement.dir/berdiri.cpp.i"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/elpistars/elpistars/Project_Humanoid/src/movement/berdiri.cpp > CMakeFiles/core_movement.dir/berdiri.cpp.i

src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core_movement.dir/berdiri.cpp.s"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/elpistars/elpistars/Project_Humanoid/src/movement/berdiri.cpp -o CMakeFiles/core_movement.dir/berdiri.cpp.s

src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o.requires:

.PHONY : src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o.requires

src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o.provides: src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o.requires
	$(MAKE) -f src/movement/CMakeFiles/core_movement.dir/build.make src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o.provides.build
.PHONY : src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o.provides

src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o.provides.build: src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o


src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o: src/movement/CMakeFiles/core_movement.dir/flags.make
src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o: ../src/movement/putarkanan.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/elpistars/elpistars/Project_Humanoid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core_movement.dir/putarkanan.cpp.o -c /home/elpistars/elpistars/Project_Humanoid/src/movement/putarkanan.cpp

src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core_movement.dir/putarkanan.cpp.i"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/elpistars/elpistars/Project_Humanoid/src/movement/putarkanan.cpp > CMakeFiles/core_movement.dir/putarkanan.cpp.i

src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core_movement.dir/putarkanan.cpp.s"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/elpistars/elpistars/Project_Humanoid/src/movement/putarkanan.cpp -o CMakeFiles/core_movement.dir/putarkanan.cpp.s

src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o.requires:

.PHONY : src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o.requires

src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o.provides: src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o.requires
	$(MAKE) -f src/movement/CMakeFiles/core_movement.dir/build.make src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o.provides.build
.PHONY : src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o.provides

src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o.provides.build: src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o


src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o: src/movement/CMakeFiles/core_movement.dir/flags.make
src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o: ../src/movement/putarkiri.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/elpistars/elpistars/Project_Humanoid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core_movement.dir/putarkiri.cpp.o -c /home/elpistars/elpistars/Project_Humanoid/src/movement/putarkiri.cpp

src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core_movement.dir/putarkiri.cpp.i"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/elpistars/elpistars/Project_Humanoid/src/movement/putarkiri.cpp > CMakeFiles/core_movement.dir/putarkiri.cpp.i

src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core_movement.dir/putarkiri.cpp.s"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/elpistars/elpistars/Project_Humanoid/src/movement/putarkiri.cpp -o CMakeFiles/core_movement.dir/putarkiri.cpp.s

src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o.requires:

.PHONY : src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o.requires

src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o.provides: src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o.requires
	$(MAKE) -f src/movement/CMakeFiles/core_movement.dir/build.make src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o.provides.build
.PHONY : src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o.provides

src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o.provides.build: src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o


src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o: src/movement/CMakeFiles/core_movement.dir/flags.make
src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o: ../src/movement/langkah_sk2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/elpistars/elpistars/Project_Humanoid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core_movement.dir/langkah_sk2.cpp.o -c /home/elpistars/elpistars/Project_Humanoid/src/movement/langkah_sk2.cpp

src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core_movement.dir/langkah_sk2.cpp.i"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/elpistars/elpistars/Project_Humanoid/src/movement/langkah_sk2.cpp > CMakeFiles/core_movement.dir/langkah_sk2.cpp.i

src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core_movement.dir/langkah_sk2.cpp.s"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/elpistars/elpistars/Project_Humanoid/src/movement/langkah_sk2.cpp -o CMakeFiles/core_movement.dir/langkah_sk2.cpp.s

src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o.requires:

.PHONY : src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o.requires

src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o.provides: src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o.requires
	$(MAKE) -f src/movement/CMakeFiles/core_movement.dir/build.make src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o.provides.build
.PHONY : src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o.provides

src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o.provides.build: src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o


# Object files for target core_movement
core_movement_OBJECTS = \
"CMakeFiles/core_movement.dir/foot_ik.cpp.o" \
"CMakeFiles/core_movement.dir/berdiri.cpp.o" \
"CMakeFiles/core_movement.dir/putarkanan.cpp.o" \
"CMakeFiles/core_movement.dir/putarkiri.cpp.o" \
"CMakeFiles/core_movement.dir/langkah_sk2.cpp.o"

# External object files for target core_movement
core_movement_EXTERNAL_OBJECTS =

src/movement/libcore_movement.a: src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o
src/movement/libcore_movement.a: src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o
src/movement/libcore_movement.a: src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o
src/movement/libcore_movement.a: src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o
src/movement/libcore_movement.a: src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o
src/movement/libcore_movement.a: src/movement/CMakeFiles/core_movement.dir/build.make
src/movement/libcore_movement.a: src/movement/CMakeFiles/core_movement.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/elpistars/elpistars/Project_Humanoid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libcore_movement.a"
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && $(CMAKE_COMMAND) -P CMakeFiles/core_movement.dir/cmake_clean_target.cmake
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/core_movement.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/movement/CMakeFiles/core_movement.dir/build: src/movement/libcore_movement.a

.PHONY : src/movement/CMakeFiles/core_movement.dir/build

src/movement/CMakeFiles/core_movement.dir/requires: src/movement/CMakeFiles/core_movement.dir/foot_ik.cpp.o.requires
src/movement/CMakeFiles/core_movement.dir/requires: src/movement/CMakeFiles/core_movement.dir/berdiri.cpp.o.requires
src/movement/CMakeFiles/core_movement.dir/requires: src/movement/CMakeFiles/core_movement.dir/putarkanan.cpp.o.requires
src/movement/CMakeFiles/core_movement.dir/requires: src/movement/CMakeFiles/core_movement.dir/putarkiri.cpp.o.requires
src/movement/CMakeFiles/core_movement.dir/requires: src/movement/CMakeFiles/core_movement.dir/langkah_sk2.cpp.o.requires

.PHONY : src/movement/CMakeFiles/core_movement.dir/requires

src/movement/CMakeFiles/core_movement.dir/clean:
	cd /home/elpistars/elpistars/Project_Humanoid/build/src/movement && $(CMAKE_COMMAND) -P CMakeFiles/core_movement.dir/cmake_clean.cmake
.PHONY : src/movement/CMakeFiles/core_movement.dir/clean

src/movement/CMakeFiles/core_movement.dir/depend:
	cd /home/elpistars/elpistars/Project_Humanoid/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/elpistars/elpistars/Project_Humanoid /home/elpistars/elpistars/Project_Humanoid/src/movement /home/elpistars/elpistars/Project_Humanoid/build /home/elpistars/elpistars/Project_Humanoid/build/src/movement /home/elpistars/elpistars/Project_Humanoid/build/src/movement/CMakeFiles/core_movement.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/movement/CMakeFiles/core_movement.dir/depend

