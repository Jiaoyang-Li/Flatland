project(ECBS)

cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")

SET (CMAKE_LINKER  "/usr/bin/ld")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall --std=c++0x")

include_directories("inc" "src")
file(GLOB SOURCES "*src/*.cpp" "*.cpp")



# Find Boost
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )


add_executable(ECBS ${SOURCES} )
target_link_libraries(ECBS ${Boost_LIBRARIES})
