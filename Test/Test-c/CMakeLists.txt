cmake_minimum_required(VERSION 3.15)
project(HGS_Test_c)
set(CMAKE_C_STANDARD 99)

add_executable(lib_test_c test.c)

IF (NOT WIN32)
  target_link_libraries(lib_test_c m)
ENDIF()
target_link_libraries(lib_test_c lib_static)

