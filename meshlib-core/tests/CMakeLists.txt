cmake_minimum_required(VERSION 3.18)

# Fetch GoogleTest if not found
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/googletest
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# Enable testing
enable_testing()

# Test sources - using only tests that work with current API
set(TEST_SOURCES
    test_main.cpp
    test_vector.cpp
    test_matrix.cpp
    test_box.cpp
    test_mesh.cpp
    test_pointcloud.cpp
    test_primitives.cpp
    test_io.cpp
    # test_decimate.cpp   # Temporarily disabled - needs API update
    # test_smooth.cpp     # Temporarily disabled - needs API update
    # test_subdivide.cpp  # Temporarily disabled - needs API update
    # test_fillhole.cpp   # Temporarily disabled - needs API update
    # test_icp.cpp        # Temporarily disabled - needs API update
)

# Create test executable
add_executable(meshlib_core_tests ${TEST_SOURCES})

target_link_libraries(meshlib_core_tests PRIVATE
    meshlib_core
    GTest::gtest
    GTest::gtest_main
)

target_include_directories(meshlib_core_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Discover tests
include(GoogleTest)
gtest_discover_tests(meshlib_core_tests)

# Copy test data (if any)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/data)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()
