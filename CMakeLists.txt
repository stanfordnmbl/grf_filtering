cmake_minimum_required(VERSION 2.8.8)

find_package(OpenSim)

if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR
        ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    set(cpp11flags "-std=c++11")
    if(APPLE)
        set(cpp11flags "${cpp11flags} -stdlib=libc++")
    endif()
    list(APPEND CMAKE_CXX_FLAGS ${cpp11flags})
endif()

include_directories("${OpenSim_INCLUDE_DIRS}")
link_directories("${OpenSim_LIB_DIR}")

add_executable(dynhop dynhop.cpp)
target_link_libraries(dynhop "${OpenSim_LIBRARIES}")

configure_file(dynhop.osim "${CMAKE_BUILD_DIR}" COPYONLY)
