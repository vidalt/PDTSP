cmake_minimum_required(VERSION 3.8)
project(veenstra)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -m64 -march=native -fPIC")

set(SOURCE_FILES
    instance.cpp
    localsearch/bs/bscache.cpp
    localsearch/bs/bsgraph.cpp
    localsearch/bs/util.cpp
    localsearch/fastmoves.cpp
    localsearch/localsearch.cpp
    localsearch/move.cpp
    localsearch/slowmoves.cpp
    operators.cpp
    solver.cpp
    application.cpp
    random.cpp
)

include_directories(src)

set(EXECUTABLE_OUTPUT_PATH "release")

add_definitions(
    -D_REENTRANT
    -DBALAS_SIMONETTI
    -DUSE_ROUNDED_DISTANCE_COMPARISON
    -DBUILD_PDP_VERSION=''
    -DBUILD_PDP_VERSION_MSG=''
)


add_executable(veenstra ${SOURCE_FILES})

find_package(Boost COMPONENTS program_options filesystem system regex)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(veenstra ${Boost_LIBRARIES})
endif ()

target_link_libraries(veenstra)
