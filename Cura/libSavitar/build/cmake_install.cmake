# Install script for directory: /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  if(EXISTS "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Savitar.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Savitar.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Savitar.so"
         RPATH "/lib")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/python3/dist-packages/Savitar.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/lib/python3/dist-packages" TYPE SHARED_LIBRARY FILES "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/Savitar.so")
  if(EXISTS "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Savitar.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Savitar.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Savitar.so"
         OLD_RPATH "::::"
         NEW_RPATH "/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/lib/python3/dist-packages/Savitar.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/libSavitar.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Savitar" TYPE FILE FILES
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/src/ThreeMFParser.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/src/Types.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/src/SceneNode.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/src/Scene.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/src/MeshData.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/src/Vertex.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/src/Face.h"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/src/SavitarExport.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Savitar/Savitar-targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Savitar/Savitar-targets.cmake"
         "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/CMakeFiles/Export/lib/cmake/Savitar/Savitar-targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Savitar/Savitar-targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Savitar/Savitar-targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Savitar" TYPE FILE FILES "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/CMakeFiles/Export/lib/cmake/Savitar/Savitar-targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Savitar" TYPE FILE FILES "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/CMakeFiles/Export/lib/cmake/Savitar/Savitar-targets-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Savitar" TYPE FILE FILES
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/SavitarConfig.cmake"
    "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/SavitarConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/pugixml/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
