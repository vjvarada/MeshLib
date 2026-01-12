# C++ Examples

add_executable(example_boolean boolean_example.cpp)
target_link_libraries(example_boolean PRIVATE meshlib_core)

add_executable(example_decimate decimate_example.cpp)
target_link_libraries(example_decimate PRIVATE meshlib_core)

add_executable(example_icp icp_example.cpp)
target_link_libraries(example_icp PRIVATE meshlib_core)
