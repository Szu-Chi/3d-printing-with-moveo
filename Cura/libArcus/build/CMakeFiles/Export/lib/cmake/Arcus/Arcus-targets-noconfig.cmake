#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Arcus" for configuration ""
set_property(TARGET Arcus APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(Arcus PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libArcus.so.1.1.0"
  IMPORTED_SONAME_NOCONFIG "libArcus.so.3"
  )

list(APPEND _IMPORT_CHECK_TARGETS Arcus )
list(APPEND _IMPORT_CHECK_FILES_FOR_Arcus "${_IMPORT_PREFIX}/lib/libArcus.so.1.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
