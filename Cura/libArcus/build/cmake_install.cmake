# Install script for directory: /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Arcus.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Arcus.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Arcus.so"
         RPATH "/usr/local/lib")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/python3/dist-packages/Arcus.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/lib/python3/dist-packages" TYPE SHARED_LIBRARY FILES "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/Arcus.so")
  if(EXISTS "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Arcus.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Arcus.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Arcus.so"
         OLD_RPATH "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build:"
         NEW_RPATH "/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Arcus.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libArcus.so.1.1.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libArcus.so.3"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libArcus.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/libArcus.so.1.1.0"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/libArcus.so.3"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/libArcus.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libArcus.so.1.1.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libArcus.so.3"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libArcus.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "::::::::::::::"
           NEW_RPATH "/usr/local/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Arcus" TYPE FILE FILES
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/Socket.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/SocketListener.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/Types.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/MessageTypeStore.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/src/Error.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/src/ArcusExport.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Arcus/Arcus-targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Arcus/Arcus-targets.cmake"
         "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles/Export/lib/cmake/Arcus/Arcus-targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Arcus/Arcus-targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Arcus/Arcus-targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Arcus" TYPE FILE FILES "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles/Export/lib/cmake/Arcus/Arcus-targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Arcus" TYPE FILE FILES "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/CMakeFiles/Export/lib/cmake/Arcus/Arcus-targets-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Arcus" TYPE FILE FILES
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/ArcusConfig.cmake"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/ArcusConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/examples/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
