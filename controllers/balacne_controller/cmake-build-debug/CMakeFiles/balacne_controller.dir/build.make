# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2021.1.3\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2021.1.3\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/balacne_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/balacne_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/balacne_controller.dir/flags.make

CMakeFiles/balacne_controller.dir/PID.cpp.obj: CMakeFiles/balacne_controller.dir/flags.make
CMakeFiles/balacne_controller.dir/PID.cpp.obj: CMakeFiles/balacne_controller.dir/includes_CXX.rsp
CMakeFiles/balacne_controller.dir/PID.cpp.obj: ../PID.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/balacne_controller.dir/PID.cpp.obj"
	C:\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\balacne_controller.dir\PID.cpp.obj -c C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\PID.cpp

CMakeFiles/balacne_controller.dir/PID.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/balacne_controller.dir/PID.cpp.i"
	C:\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\PID.cpp > CMakeFiles\balacne_controller.dir\PID.cpp.i

CMakeFiles/balacne_controller.dir/PID.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/balacne_controller.dir/PID.cpp.s"
	C:\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\PID.cpp -o CMakeFiles\balacne_controller.dir\PID.cpp.s

CMakeFiles/balacne_controller.dir/balacne_controller.cpp.obj: CMakeFiles/balacne_controller.dir/flags.make
CMakeFiles/balacne_controller.dir/balacne_controller.cpp.obj: CMakeFiles/balacne_controller.dir/includes_CXX.rsp
CMakeFiles/balacne_controller.dir/balacne_controller.cpp.obj: ../balacne_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/balacne_controller.dir/balacne_controller.cpp.obj"
	C:\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\balacne_controller.dir\balacne_controller.cpp.obj -c C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\balacne_controller.cpp

CMakeFiles/balacne_controller.dir/balacne_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/balacne_controller.dir/balacne_controller.cpp.i"
	C:\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\balacne_controller.cpp > CMakeFiles\balacne_controller.dir\balacne_controller.cpp.i

CMakeFiles/balacne_controller.dir/balacne_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/balacne_controller.dir/balacne_controller.cpp.s"
	C:\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\balacne_controller.cpp -o CMakeFiles\balacne_controller.dir\balacne_controller.cpp.s

# Object files for target balacne_controller
balacne_controller_OBJECTS = \
"CMakeFiles/balacne_controller.dir/PID.cpp.obj" \
"CMakeFiles/balacne_controller.dir/balacne_controller.cpp.obj"

# External object files for target balacne_controller
balacne_controller_EXTERNAL_OBJECTS =

balacne_controller.exe: CMakeFiles/balacne_controller.dir/PID.cpp.obj
balacne_controller.exe: CMakeFiles/balacne_controller.dir/balacne_controller.cpp.obj
balacne_controller.exe: CMakeFiles/balacne_controller.dir/build.make
balacne_controller.exe: CMakeFiles/balacne_controller.dir/linklibs.rsp
balacne_controller.exe: CMakeFiles/balacne_controller.dir/objects1.rsp
balacne_controller.exe: CMakeFiles/balacne_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable balacne_controller.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\balacne_controller.dir\link.txt --verbose=$(VERBOSE)
	"C:\Program Files\JetBrains\CLion 2021.1.3\bin\cmake\win\bin\cmake.exe" -E copy C:/Users/Administrator/Desktop/hard/balance_car/controllers/balacne_controller/cmake-build-debug/balacne_controller.exe C:/Users/Administrator/Desktop/hard/balance_car/controllers/balacne_controller

# Rule to build all files generated by this target.
CMakeFiles/balacne_controller.dir/build: balacne_controller.exe

.PHONY : CMakeFiles/balacne_controller.dir/build

CMakeFiles/balacne_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\balacne_controller.dir\cmake_clean.cmake
.PHONY : CMakeFiles/balacne_controller.dir/clean

CMakeFiles/balacne_controller.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\cmake-build-debug C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\cmake-build-debug C:\Users\Administrator\Desktop\hard\balance_car\controllers\balacne_controller\cmake-build-debug\CMakeFiles\balacne_controller.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/balacne_controller.dir/depend

