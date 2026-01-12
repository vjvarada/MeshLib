#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "MeshLibCore::meshlib_core" for configuration "MinSizeRel"
set_property(TARGET MeshLibCore::meshlib_core APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(MeshLibCore::meshlib_core PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/meshlib_core.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/meshlib_core.dll"
  )

list(APPEND _cmake_import_check_targets MeshLibCore::meshlib_core )
list(APPEND _cmake_import_check_files_for_MeshLibCore::meshlib_core "${_IMPORT_PREFIX}/lib/meshlib_core.lib" "${_IMPORT_PREFIX}/bin/meshlib_core.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
