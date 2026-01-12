#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "MeshLibCore::meshlib_c" for configuration "RelWithDebInfo"
set_property(TARGET MeshLibCore::meshlib_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(MeshLibCore::meshlib_c PROPERTIES
  IMPORTED_IMPLIB_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/meshlib_c.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELWITHDEBINFO "MeshLibCore::meshlib_core"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/bin/meshlib_c.dll"
  )

list(APPEND _cmake_import_check_targets MeshLibCore::meshlib_c )
list(APPEND _cmake_import_check_files_for_MeshLibCore::meshlib_c "${_IMPORT_PREFIX}/lib/meshlib_c.lib" "${_IMPORT_PREFIX}/bin/meshlib_c.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
