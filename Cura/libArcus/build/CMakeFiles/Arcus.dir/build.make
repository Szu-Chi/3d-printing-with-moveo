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
CMAKE_SOURCE_DIR = /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build

# Include any dependencies generated for this target.
include CMakeFiles/Arcus.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Arcus.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Arcus.dir/flags.make

CMakeFiles/Arcus.dir/src/Socket.cpp.o: CMakeFiles/Arcus.dir/flags.make
CMakeFiles/Arcus.dir/src/Socket.cpp.o: ../src/Socket.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Arcus.dir/src/Socket.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Arcus.dir/src/Socket.cpp.o -c /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/Socket.cpp

CMakeFiles/Arcus.dir/src/Socket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Arcus.dir/src/Socket.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/Socket.cpp > CMakeFiles/Arcus.dir/src/Socket.cpp.i

CMakeFiles/Arcus.dir/src/Socket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Arcus.dir/src/Socket.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/Socket.cpp -o CMakeFiles/Arcus.dir/src/Socket.cpp.s

CMakeFiles/Arcus.dir/src/Socket.cpp.o.requires:

.PHONY : CMakeFiles/Arcus.dir/src/Socket.cpp.o.requires

CMakeFiles/Arcus.dir/src/Socket.cpp.o.provides: CMakeFiles/Arcus.dir/src/Socket.cpp.o.requires
	$(MAKE) -f CMakeFiles/Arcus.dir/build.make CMakeFiles/Arcus.dir/src/Socket.cpp.o.provides.build
.PHONY : CMakeFiles/Arcus.dir/src/Socket.cpp.o.provides

CMakeFiles/Arcus.dir/src/Socket.cpp.o.provides.build: CMakeFiles/Arcus.dir/src/Socket.cpp.o


CMakeFiles/Arcus.dir/src/SocketListener.cpp.o: CMakeFiles/Arcus.dir/flags.make
CMakeFiles/Arcus.dir/src/SocketListener.cpp.o: ../src/SocketListener.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Arcus.dir/src/SocketListener.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Arcus.dir/src/SocketListener.cpp.o -c /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/SocketListener.cpp

CMakeFiles/Arcus.dir/src/SocketListener.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Arcus.dir/src/SocketListener.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/SocketListener.cpp > CMakeFiles/Arcus.dir/src/SocketListener.cpp.i

CMakeFiles/Arcus.dir/src/SocketListener.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Arcus.dir/src/SocketListener.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/SocketListener.cpp -o CMakeFiles/Arcus.dir/src/SocketListener.cpp.s

CMakeFiles/Arcus.dir/src/SocketListener.cpp.o.requires:

.PHONY : CMakeFiles/Arcus.dir/src/SocketListener.cpp.o.requires

CMakeFiles/Arcus.dir/src/SocketListener.cpp.o.provides: CMakeFiles/Arcus.dir/src/SocketListener.cpp.o.requires
	$(MAKE) -f CMakeFiles/Arcus.dir/build.make CMakeFiles/Arcus.dir/src/SocketListener.cpp.o.provides.build
.PHONY : CMakeFiles/Arcus.dir/src/SocketListener.cpp.o.provides

CMakeFiles/Arcus.dir/src/SocketListener.cpp.o.provides.build: CMakeFiles/Arcus.dir/src/SocketListener.cpp.o


CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o: CMakeFiles/Arcus.dir/flags.make
CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o: ../src/MessageTypeStore.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o -c /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/MessageTypeStore.cpp

CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/MessageTypeStore.cpp > CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.i

CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/MessageTypeStore.cpp -o CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.s

CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o.requires:

.PHONY : CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o.requires

CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o.provides: CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o.requires
	$(MAKE) -f CMakeFiles/Arcus.dir/build.make CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o.provides.build
.PHONY : CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o.provides

CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o.provides.build: CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o


CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o: CMakeFiles/Arcus.dir/flags.make
CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o: ../src/PlatformSocket.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o -c /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/PlatformSocket.cpp

CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/PlatformSocket.cpp > CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.i

CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/PlatformSocket.cpp -o CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.s

CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o.requires:

.PHONY : CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o.requires

CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o.provides: CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o.requires
	$(MAKE) -f CMakeFiles/Arcus.dir/build.make CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o.provides.build
.PHONY : CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o.provides

CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o.provides.build: CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o


CMakeFiles/Arcus.dir/src/Error.cpp.o: CMakeFiles/Arcus.dir/flags.make
CMakeFiles/Arcus.dir/src/Error.cpp.o: ../src/Error.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Arcus.dir/src/Error.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Arcus.dir/src/Error.cpp.o -c /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/Error.cpp

CMakeFiles/Arcus.dir/src/Error.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Arcus.dir/src/Error.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/Error.cpp > CMakeFiles/Arcus.dir/src/Error.cpp.i

CMakeFiles/Arcus.dir/src/Error.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Arcus.dir/src/Error.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/Error.cpp -o CMakeFiles/Arcus.dir/src/Error.cpp.s

CMakeFiles/Arcus.dir/src/Error.cpp.o.requires:

.PHONY : CMakeFiles/Arcus.dir/src/Error.cpp.o.requires

CMakeFiles/Arcus.dir/src/Error.cpp.o.provides: CMakeFiles/Arcus.dir/src/Error.cpp.o.requires
	$(MAKE) -f CMakeFiles/Arcus.dir/build.make CMakeFiles/Arcus.dir/src/Error.cpp.o.provides.build
.PHONY : CMakeFiles/Arcus.dir/src/Error.cpp.o.provides

CMakeFiles/Arcus.dir/src/Error.cpp.o.provides.build: CMakeFiles/Arcus.dir/src/Error.cpp.o


# Object files for target Arcus
Arcus_OBJECTS = \
"CMakeFiles/Arcus.dir/src/Socket.cpp.o" \
"CMakeFiles/Arcus.dir/src/SocketListener.cpp.o" \
"CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o" \
"CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o" \
"CMakeFiles/Arcus.dir/src/Error.cpp.o"

# External object files for target Arcus
Arcus_EXTERNAL_OBJECTS =

libArcus.so.1.1.0: CMakeFiles/Arcus.dir/src/Socket.cpp.o
libArcus.so.1.1.0: CMakeFiles/Arcus.dir/src/SocketListener.cpp.o
libArcus.so.1.1.0: CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o
libArcus.so.1.1.0: CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o
libArcus.so.1.1.0: CMakeFiles/Arcus.dir/src/Error.cpp.o
libArcus.so.1.1.0: CMakeFiles/Arcus.dir/build.make
libArcus.so.1.1.0: /usr/local/lib/libprotobuf.a
libArcus.so.1.1.0: CMakeFiles/Arcus.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library libArcus.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Arcus.dir/link.txt --verbose=$(VERBOSE)
	$(CMAKE_COMMAND) -E cmake_symlink_library libArcus.so.1.1.0 libArcus.so.3 libArcus.so

libArcus.so.3: libArcus.so.1.1.0
	@$(CMAKE_COMMAND) -E touch_nocreate libArcus.so.3

libArcus.so: libArcus.so.1.1.0
	@$(CMAKE_COMMAND) -E touch_nocreate libArcus.so

# Rule to build all files generated by this target.
CMakeFiles/Arcus.dir/build: libArcus.so

.PHONY : CMakeFiles/Arcus.dir/build

CMakeFiles/Arcus.dir/requires: CMakeFiles/Arcus.dir/src/Socket.cpp.o.requires
CMakeFiles/Arcus.dir/requires: CMakeFiles/Arcus.dir/src/SocketListener.cpp.o.requires
CMakeFiles/Arcus.dir/requires: CMakeFiles/Arcus.dir/src/MessageTypeStore.cpp.o.requires
CMakeFiles/Arcus.dir/requires: CMakeFiles/Arcus.dir/src/PlatformSocket.cpp.o.requires
CMakeFiles/Arcus.dir/requires: CMakeFiles/Arcus.dir/src/Error.cpp.o.requires

.PHONY : CMakeFiles/Arcus.dir/requires

CMakeFiles/Arcus.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Arcus.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Arcus.dir/clean

CMakeFiles/Arcus.dir/depend:
	cd /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles/Arcus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Arcus.dir/depend
