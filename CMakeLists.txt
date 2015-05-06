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
configure_file(create_experimental_data.py "${CMAKE_BUILD_DIR}" COPYONLY)
configure_file(id_setup.xml "${CMAKE_BUILD_DIR}" COPYONLY)
configure_file(ik_setup.xml "${CMAKE_BUILD_DIR}" COPYONLY)
configure_file(ik_taskset.xml "${CMAKE_BUILD_DIR}" COPYONLY)
configure_file(cmc_setup.xml "${CMAKE_BUILD_DIR}" COPYONLY)
configure_file(cmc_tasks.xml "${CMAKE_BUILD_DIR}" COPYONLY)
configure_file(cmc_actuators.xml "${CMAKE_BUILD_DIR}" COPYONLY)
configure_file(external_loads.xml "${CMAKE_BUILD_DIR}" COPYONLY)

add_custom_command(OUTPUT ${CMAKE_BUILD_DIR}/ik_solution.mot
    COMMAND ${OpenSim_ROOT_DIR}/bin/ik -S ik_setup.xml
    )
add_custom_target(ik DEPENDS ${CMAKE_BUILD_DIR}/ik_solution.mot)

add_custom_command(OUTPUT ${CMAKE_BUILD_DIR}/id_solution.mot
    COMMAND ${OpenSim_ROOT_DIR}/bin/id -S id_setup.xml
    )
add_custom_target(id DEPENDS ${CMAKE_BUILD_DIR}/id_solution.mot)

add_custom_command(OUTPUT ${CMAKE_BUILD_DIR}/results/dynhop_cmc_states.sto
    COMMAND ${OpenSim_ROOT_DIR}/bin/cmc -S cmc_setup.xml
    )
add_custom_target(cmc DEPENDS ${CMAKE_BUILD_DIR}/results/dynhop_cmc_states.sto)
