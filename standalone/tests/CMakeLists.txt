project(MRMeshTests CXX)

add_executable(test_mrmesh_basic test_mrmesh_basic.cpp)

target_link_libraries(test_mrmesh_basic PRIVATE MRMesh)

target_include_directories(test_mrmesh_basic PRIVATE
    ${CMAKE_SOURCE_DIR}/source
)

enable_testing()
add_test(NAME BasicMeshTest COMMAND test_mrmesh_basic)

# MRVoxels test
if(MESHLIB_BUILD_VOXELS)
    add_executable(test_mrvoxels_basic test_mrvoxels_basic.cpp)
    
    target_link_libraries(test_mrvoxels_basic PRIVATE MRVoxels MRMesh)
    
    target_include_directories(test_mrvoxels_basic PRIVATE
        ${CMAKE_SOURCE_DIR}/source
    )
    
    add_test(NAME BasicVoxelsTest COMMAND test_mrvoxels_basic)
endif()
