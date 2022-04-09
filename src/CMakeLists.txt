cmake_minimum_required(VERSION 3.10)

include_directories(${MAD-RT_SOURCE_DIR}/include)
file(GLOB SOURCES ${MAD-RT_SOURCE_DIR}/include/*.h ${MAD-RT_SOURCE_DIR}/src/*.cpp)
add_executable(mad_rt ${SOURCES})
