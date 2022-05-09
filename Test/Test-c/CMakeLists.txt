cmake_minimum_required(VERSION 3.15)
project(HGS_Test_c)
set(CMAKE_C_STANDARD 99)

include_directories(${CMAKE_SOURCE_DIR}/Program)

add_executable(lib_test_c
        test.c
        ${CMAKE_SOURCE_DIR}/Program/C_Interface.h
        ${CMAKE_SOURCE_DIR}/Program/AlgorithmParameters.h
        )

target_link_libraries(lib_test_c -lm lib)  # need -lm to link math library

