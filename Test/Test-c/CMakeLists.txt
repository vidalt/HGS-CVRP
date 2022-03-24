cmake_minimum_required(VERSION 3.15)
project(HGS_Test)
set(CMAKE_C_STANDARD 99)

include_directories(${CMAKE_SOURCE_DIR}/Program)

add_executable(test_c_interface
        test.c
        ${CMAKE_SOURCE_DIR}/Program/C_Interface.h
        ${CMAKE_SOURCE_DIR}/Program/AlgorithmParameters.h
        )

target_link_libraries(test_c_interface lib)

