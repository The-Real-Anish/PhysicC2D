cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)

# find all source files
file(GLOB_RECURSE SOURCES
	src/*.cpp
)

add_executable(PhysicC2D ${SOURCES})

add_subdirectory(ext/glm)

target_link_libraries(PhysicC2D glm)

target_include_directories(PhysicC2D PUBLIC inc)