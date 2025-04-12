cmake_minimum_required(VERSION 3.10)

project(rix-rdf)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie")
add_compile_options(-Wno-deprecated)

if(DEFINED LOGLEVEL)
    add_compile_definitions(RIX_UTIL_LOG_LEVEL=${LOGLEVEL})
else()
    add_compile_definitions(RIX_UTIL_LOG_LEVEL=1)
endif()

find_library(PTHREAD pthread REQUIRED)

add_subdirectory(googletest)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# Define include directories
set(INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/eigen/
    ${CMAKE_SOURCE_DIR}/json/include
)

# Define libraries
set(RIX_LIBS
    ${CMAKE_SOURCE_DIR}/lib/librixrdf.a
    ${CMAKE_SOURCE_DIR}/lib/librixcore.a
    ${CMAKE_SOURCE_DIR}/lib/librixutil.a
    ${CMAKE_SOURCE_DIR}/lib/librixipc.a
)

### IKSRV ###
add_executable(iksrv src/iksrv.cpp src/ik_solver.cpp)
target_include_directories(iksrv PRIVATE ${INCLUDE_DIRS})
target_link_libraries(iksrv ${RIX_LIBS} pthread)

### UNIT TESTS ###
add_executable(test_ik_solver
    test/unit_tests/test_ik_solver.cpp
    src/ik_solver.cpp
)
target_include_directories(test_ik_solver PRIVATE ${INCLUDE_DIRS} test/include/)
target_link_libraries(test_ik_solver ${RIX_LIBS} pthread GTest::gtest_main)

### MANUAL TESTS ###
add_executable(test_iksrv
    test/manual_tests/test_iksrv.cpp
    src/ik_solver.cpp
)
target_include_directories(test_iksrv PRIVATE ${INCLUDE_DIRS} test/include/)
target_link_libraries(test_iksrv ${RIX_LIBS} pthread GTest::gtest_main)