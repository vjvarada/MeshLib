project(MRIOExtras CXX)

option(MRIOEXTRAS_NO_3MF "Disable 3MF support" OFF)
option(MRIOEXTRAS_NO_CTM "Disable OpenCTM support" OFF)
option(MRIOEXTRAS_NO_E57 "Disable E57 support" OFF)
option(MRIOEXTRAS_NO_GLTF "Disable glTF support" OFF)
option(MRIOEXTRAS_NO_JPEG "Disable JPEG support" OFF)
option(MRIOEXTRAS_NO_LAS "Disable LAS/LAZ support" OFF)
option(MRIOEXTRAS_NO_PDF "Disable PDF support" OFF)
option(MRIOEXTRAS_NO_PNG "Disable PNG support" OFF)
option(MRIOEXTRAS_NO_STEP "Disable STEP support" OFF)
option(MRIOEXTRAS_NO_TIFF "Disable TIFF support" OFF)
option(MRIOEXTRAS_NO_ZLIB "Disable Zlib usage" OFF)

option(MRIOEXTRAS_OPENCASCADE_USE_XDE "Use OpenCASCADE XDE module" ON)

IF(MR_EMSCRIPTEN)
  set(MRIOEXTRAS_NO_PDF ON)
  set(MRIOEXTRAS_NO_STEP ON)
  set(MRIOEXTRAS_NO_TIFF ON)
ENDIF()

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.h")

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config_cmake.h.in ${CMAKE_CURRENT_SOURCE_DIR}/config_cmake.h)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    MRMesh
)

# Add tinygltf include directory if available
IF(NOT MRIOEXTRAS_NO_GLTF AND EXISTS "${MESHLIB_THIRDPARTY_DIR}/tinygltf")
  target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${MESHLIB_THIRDPARTY_DIR}/tinygltf>)
ENDIF()

IF(NOT MRIOEXTRAS_NO_3MF)
  find_package(tinyxml2 CONFIG REQUIRED)
  target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2::tinyxml2)
ENDIF()

IF(NOT MRIOEXTRAS_NO_CTM)
  target_link_libraries(${PROJECT_NAME} PRIVATE OpenCTM)
  IF(NOT MR_EMSCRIPTEN)
    target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${MESHLIB_THIRDPARTY_DIR}/OpenCTM-git/lib>)
  ENDIF()
ENDIF()

IF(NOT MRIOEXTRAS_NO_E57)
  find_package(E57Format CONFIG REQUIRED)
  target_link_libraries(${PROJECT_NAME} PRIVATE E57Format)
ENDIF()

IF(NOT MRIOEXTRAS_NO_JPEG)
  IF(APPLE)
    target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${HOMEBREW_PREFIX}/opt/jpeg-turbo/include>)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${HOMEBREW_PREFIX}/opt/jpeg-turbo/lib/libturbojpeg.dylib)
  ELSE()
    find_package(libjpeg-turbo CONFIG REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE $<IF:$<TARGET_EXISTS:libjpeg-turbo::turbojpeg>,libjpeg-turbo::turbojpeg,libjpeg-turbo::turbojpeg-static>)
  ENDIF()
ENDIF()

IF(NOT MRIOEXTRAS_NO_LAS)
  target_link_libraries(${PROJECT_NAME} PRIVATE lazperf)
  IF(NOT MR_EMSCRIPTEN)
    target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${MESHLIB_THIRDPARTY_DIR}/laz-perf/cpp>)
  ENDIF()
ENDIF()

IF(NOT MRIOEXTRAS_NO_PDF)
  find_package(unofficial-libharu CONFIG REQUIRED)
  target_link_libraries(${PROJECT_NAME} PRIVATE unofficial::libharu::hpdf)
ENDIF()

IF(NOT MRIOEXTRAS_NO_PNG)
  IF(MR_EMSCRIPTEN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s USE_LIBPNG=1")
  ELSE()
    find_package(PNG REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE PNG::PNG)
  ENDIF()
ENDIF()

IF(NOT MRIOEXTRAS_NO_STEP)
  find_package(OpenCASCADE REQUIRED
    COMPONENTS
    # NOTE: the component order is significant
    FoundationClasses
    ModelingData
    ModelingAlgorithms
    Visualization
    ApplicationFramework
    DataExchange
  )
  target_include_directories(${PROJECT_NAME} BEFORE PRIVATE ${OpenCASCADE_INCLUDE_DIR})
  target_link_directories(${PROJECT_NAME} BEFORE PRIVATE ${OpenCASCADE_LIBRARY_DIR})

  set(OpenCASCADE_REQUIRED_LIBRARIES
    TKernel
    TKBRep
    TKMath
    TKMesh
    TKDESTEP
    TKSTEP
    TKTopAlgo
    TKXSBase
  )
  IF(MRIOEXTRAS_OPENCASCADE_USE_XDE)
    list(APPEND OpenCASCADE_REQUIRED_LIBRARIES
      TKXDESTEP
      TKLCAF
      TKXCAF
    )
  ENDIF()
  foreach(LIBRARY_NAME ${OpenCASCADE_LIBRARIES})
    IF(${LIBRARY_NAME} IN_LIST OpenCASCADE_REQUIRED_LIBRARIES)
      target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARY_NAME})
    ENDIF()
  endforeach()
ENDIF()

IF(NOT MRIOEXTRAS_NO_TIFF)
  find_package(TIFF REQUIRED)
  target_link_libraries(${PROJECT_NAME} PRIVATE TIFF::TIFF)
ENDIF()

IF(NOT MRIOEXTRAS_NO_ZLIB)
  find_package(ZLIB REQUIRED)
  target_link_libraries(${PROJECT_NAME} PRIVATE ZLIB::ZLIB)
ENDIF()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION "${MR_MAIN_LIB_DIR}"
  ARCHIVE DESTINATION "${MR_MAIN_LIB_DIR}"
  RUNTIME DESTINATION "${MR_BIN_DIR}"
)

install(
  FILES ${HEADERS}
  DESTINATION "${MR_INCLUDE_DIR}/${PROJECT_NAME}"
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION ${MR_CONFIG_DIR}
)

install(
  EXPORT ${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE MeshLib::
  DESTINATION ${MR_CONFIG_DIR}
)
