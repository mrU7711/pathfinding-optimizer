# Pathfinding Optimizer

A C++ implementation of classical pathfinding algorithms for robot navigation on 2D grid maps. Implements A*, Dijkstra's algorithm, and Rapidly-exploring Random Trees (RRT).

## Overview

This project provides optimized implementations of three fundamental path planning algorithms used in robotics and autonomous navigation. Each algorithm operates on a 2D occupancy grid map with configurable obstacles.

## Project Structure

```
pathfinding-optimizer/
├── src/
│   ├── main.cpp            # Entry point and CLI
│   ├── grid_map.h/cpp      # 2D occupancy grid
│   ├── astar.h/cpp         # A* pathfinding
│   ├── dijkstra.h/cpp      # Dijkstra's algorithm
│   ├── rrt.h/cpp           # RRT path planning
│   └── utils.h/cpp         # Distance and smoothing utilities
├── tests/
│   ├── test_astar.cpp      # A* unit tests
│   └── test_dijkstra.cpp   # Dijkstra unit tests
├── config/
│   └── map_config.txt      # Grid and obstacle config
├── CMakeLists.txt
├── .gitignore
└── README.md
```

## Building

```bash
mkdir build && cd build
cmake ..
make
```

## Usage

```bash
./pathfinder --algorithm astar --start 0,0 --goal 19,14 --map ../config/map_config.txt
./pathfinder --algorithm dijkstra --start 0,0 --goal 19,14 --map ../config/map_config.txt
./pathfinder --algorithm rrt --start 0,0 --goal 19,14 --map ../config/map_config.txt
```

## Algorithms

### A* Search
- Uses Manhattan distance heuristic
- Guarantees optimal path
- Time complexity: O(E log V)

### Dijkstra's Algorithm
- Uniform cost search (A* with h=0)
- Explores all directions equally
- Guarantees shortest path

### RRT (Rapidly-exploring Random Tree)
- Probabilistic sampling-based planner
- Handles high-dimensional spaces
- Does not guarantee optimality

## Algorithm Comparison

| Algorithm | Optimal | Complete | Time Complexity |
|-----------|---------|----------|-----------------|
| A*        | Yes     | Yes      | O(E log V)      |
| Dijkstra  | Yes     | Yes      | O(V^2) or O(E log V) |
| RRT       | No      | Probabilistically | O(n)    |

## Running Tests

```bash
cd build
./test_astar
./test_dijkstra
```

## License

MIT License
