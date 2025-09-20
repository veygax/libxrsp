# Samples CMakeLists.txt

add_executable(sample_pair_and_echo pair_and_echo.cc)

target_link_libraries(sample_pair_and_echo PRIVATE xrsp)
