cmake_minimum_required(VERSION 3.15)
project(HGS_CVRP)
set(CMAKE_CXX_STANDARD 11)

set(
        src_files
        Program/Genetic.cpp
        Program/Individual.cpp
        Program/LocalSearch.cpp
        Program/Params.cpp
        Program/Population.cpp
        Program/Split.cpp
        Program/CVRPLIB.cpp
        Program/AlgorithmParameters.cpp
)

include_directories(Program)
#include_directories(Interface)

# Build Executable
add_executable(bin
        Program/main.cpp
        ${src_files}
        )
set_target_properties(bin PROPERTIES OUTPUT_NAME hgs)

# Test Executable
include(CTest)
add_test(NAME    test_X-n101-k25
         COMMAND ${CMAKE_COMMAND}   -DINSTANCE=X-n101-k25
                                    -DCOST=27591
                                    -P ${CMAKE_CURRENT_SOURCE_DIR}/TestExecutable.cmake)
add_test(NAME    test_X-n106-k14
         COMMAND ${CMAKE_COMMAND}   -DINSTANCE=X-n110-k13
                                    -DCOST=14971
                                    -P ${CMAKE_CURRENT_SOURCE_DIR}/TestExecutable.cmake)


# Build Library
#add_library(lib SHARED
#        ${src_files}
#        Interface/C_Interface.cpp
#        Interface/C_Interface.h
#        )
#set_target_properties(lib PROPERTIES OUTPUT_NAME hgscvrp)

# Test Library
#add_subdirectory(Test)
#enable_testing()
#add_test(NAME c_test
#        COMMAND c_test)
