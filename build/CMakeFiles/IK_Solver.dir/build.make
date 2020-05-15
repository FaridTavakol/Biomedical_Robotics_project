# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /usr/local/lib/python3.6/dist-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /usr/local/lib/python3.6/dist-packages/cmake/data/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/build"

# Include any dependencies generated for this target.
include CMakeFiles/IK_Solver.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/IK_Solver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/IK_Solver.dir/flags.make

CMakeFiles/IK_Solver.dir/IK_Solver.cpp.o: CMakeFiles/IK_Solver.dir/flags.make
CMakeFiles/IK_Solver.dir/IK_Solver.cpp.o: ../IK_Solver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/IK_Solver.dir/IK_Solver.cpp.o"
	/usr/bin/clang++-6.0  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/IK_Solver.dir/IK_Solver.cpp.o -c "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/IK_Solver.cpp"

CMakeFiles/IK_Solver.dir/IK_Solver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IK_Solver.dir/IK_Solver.cpp.i"
	/usr/bin/clang++-6.0 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/IK_Solver.cpp" > CMakeFiles/IK_Solver.dir/IK_Solver.cpp.i

CMakeFiles/IK_Solver.dir/IK_Solver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IK_Solver.dir/IK_Solver.cpp.s"
	/usr/bin/clang++-6.0 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/IK_Solver.cpp" -o CMakeFiles/IK_Solver.dir/IK_Solver.cpp.s

# Object files for target IK_Solver
IK_Solver_OBJECTS = \
"CMakeFiles/IK_Solver.dir/IK_Solver.cpp.o"

# External object files for target IK_Solver
IK_Solver_EXTERNAL_OBJECTS =

IK_Solver: CMakeFiles/IK_Solver.dir/IK_Solver.cpp.o
IK_Solver: CMakeFiles/IK_Solver.dir/build.make
IK_Solver: libNeuroKinematics.a
IK_Solver: CMakeFiles/IK_Solver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable IK_Solver"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/IK_Solver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/IK_Solver.dir/build: IK_Solver

.PHONY : CMakeFiles/IK_Solver.dir/build

CMakeFiles/IK_Solver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/IK_Solver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/IK_Solver.dir/clean

CMakeFiles/IK_Solver.dir/depend:
	cd "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics" "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics" "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/build" "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/build" "/home/aimlab/Documents/Courses/Biomedical Engineering/project/Neuro_code/Kinematics/build/CMakeFiles/IK_Solver.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/IK_Solver.dir/depend

