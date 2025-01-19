cmake_minimum_required(VERSION 3.14)
project(PathfindingOptimizer VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(pathfinder
    src/main.cpp
    src/grid_map.cpp
    src/astar.cpp
    src/dijkstra.cpp
    src/rrt.cpp
    src/utils.cpp
)

target_include_directories(pathfinder PRIVATE src)

# Test executables
add_executable(test_astar tests/test_astar.cpp src/astar.cpp src/grid_map.cpp src/utils.cpp)
target_include_directories(test_astar PRIVATE src)

add_executable(test_dijkstra tests/test_dijkstra.cpp src/dijkstra.cpp src/grid_map.cpp src/utils.cpp)
target_include_directories(test_dijkstra PRIVATE src)
