cmake_minimum_required(VERSION 3.14)
project(PathfindingVisualizer LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()


find_package(raylib 5.0 QUIET)
if(NOT raylib_FOUND)
    include(FetchContent)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG        5.5
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(raylib)
endif()

add_executable(pathfinding_visualizer src/main.cpp)
target_link_libraries(pathfinding_visualizer PRIVATE raylib)
if(MSVC)
    target_compile_options(pathfinding_visualizer PRIVATE /W4)
else()
    target_compile_options(pathfinding_visualizer PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_executable(test_algorithms tests/test_algorithms.cpp)
if(NOT MSVC)
    target_compile_options(test_algorithms PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME algorithms COMMAND test_algorithms)
