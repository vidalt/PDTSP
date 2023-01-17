cmake_minimum_required(VERSION 3.15)
project(pdtsp)

set(EXECUTABLE_OUTPUT_PATH "./")
if(NOT MSVC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -m64 -march=native -fPIC")
endif()

add_definitions(
    -D_REENTRANT
    -DBALAS_SIMONETTI
    -DBUILD_PDP_VERSION=''
    -DBUILD_PDP_VERSION_MSG=''
)

set(SOURCE_FILES_HGS
    PDP-HGS/main.cpp
    PDP-HGS/utils/application.cpp
    PDP-HGS/utils/random.cpp
    PDP-HGS/hgsadc/adcpopulation.cpp
    PDP-HGS/hgsadc/hgsadc.cpp
    PDP-HGS/pdp/instancereader.cpp
    PDP-HGS/pdp/pdpeducate.cpp
    PDP-HGS/pdp/pdpinstance.cpp
    PDP-HGS/pdp/pdproute.cpp
    PDP-HGS/pdp/pdprouteinfo.cpp
    PDP-HGS/pdp/pdpsolution.cpp
    PDP-HGS/pdp/moves/balas_simonetti/bscache.cpp
    PDP-HGS/pdp/moves/balas_simonetti/bsgraph.cpp
    PDP-HGS/pdp/moves/balas_simonetti/util.cpp
    PDP-HGS/pdp/moves/pdp2koptmove.cpp
    PDP-HGS/pdp/moves/pdp2optmove.cpp
    PDP-HGS/pdp/moves/pdp4optmove.cpp
    PDP-HGS/pdp/moves/pdpbsmove.cpp
    PDP-HGS/pdp/moves/pdpmoveevaluation.cpp
    PDP-HGS/pdp/moves/pdporoptmove.cpp
    PDP-HGS/pdp/moves/pdprelocatemove.cpp

)

set(SOURCE_FILES_RR
    PDP-RR/application.cpp
    PDP-RR/instance.cpp
    PDP-RR/operators.cpp
    PDP-RR/random.cpp
    PDP-RR/solver.cpp
)

include_directories(pdphgs PDP-HGS)
include_directories(pdprr PDP-RR)

add_executable(pdphgs ${SOURCE_FILES_HGS})
add_executable(pdprr ${SOURCE_FILES_RR})
if(DEFINED ENV{BOOST_ROOT})
set(Boost_ROOT $ENV{BOOST_ROOT})
elseif(DEFINED ENV{BOOSTROOT})
set(Boost_ROOT $ENV{BOOSTROOT})
else()
return()
endif()
file(GLOB BOOST_SEARCH_PATHS $ENV{Boost_ROOT}/stage/lib/cmake/Boost-*)
find_path(Boost_DIR NAMES BoostConfig.cmake PATHS ${BOOST_SEARCH_PATHS})
find_package(Boost COMPONENTS program_options filesystem system regex)
if (Boost_FOUND)
    include_directories(${BOOST_INCLUDE_DIRS})

    target_link_libraries(pdphgs ${Boost_LIBRARIES})
    target_link_libraries(pdprr ${Boost_LIBRARIES})
endif ()

target_link_libraries(pdphgs)
target_link_libraries(pdprr)
