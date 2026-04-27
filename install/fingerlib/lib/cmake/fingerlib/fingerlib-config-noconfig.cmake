#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fingerlib::fingerlib" for configuration ""
set_property(TARGET fingerlib::fingerlib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(fingerlib::fingerlib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libfingerlib.a"
  )

list(APPEND _cmake_import_check_targets fingerlib::fingerlib )
list(APPEND _cmake_import_check_files_for_fingerlib::fingerlib "${_IMPORT_PREFIX}/lib/libfingerlib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
