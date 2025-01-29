#include <iostream>
#include <cassert>
#include "../src/grid_map.h"
#include "../src/astar.h"
#include "../src/utils.h"

void testBasicPath() {
    GridMap map(10, 10);
    AStar astar(map);
    auto result = astar.findPath(0, 0, 9, 9);
    assert(result.found && "A* should find path in empty grid");
    assert(result.path.front() == std::make_pair(0, 0));
    assert(result.path.back() == std::make_pair(9, 9));
    std::cout << "[PASS] testBasicPath" << std::endl;
}

void testBlockedPath() {
    GridMap map(5, 5);
    // Create wall blocking all paths
    for (int y = 0; y < 5; ++y) {
        map.setObstacle(2, y);
    }
    AStar astar(map);
    auto result = astar.findPath(0, 0, 4, 4);
    assert(!result.found && "A* should not find path through solid wall");
    std::cout << "[PASS] testBlockedPath" << std::endl;
}

void testStartEqualsGoal() {
    GridMap map(10, 10);
    AStar astar(map);
    auto result = astar.findPath(5, 5, 5, 5);
    assert(result.found && "A* should handle start==goal");
    std::cout << "[PASS] testStartEqualsGoal" << std::endl;
}

void testPathAroundObstacle() {
    GridMap map(10, 10);
    map.setObstacle(5, 4);
    map.setObstacle(5, 5);
    map.setObstacle(5, 6);
    AStar astar(map);
    auto result = astar.findPath(3, 5, 7, 5);
    assert(result.found && "A* should find path around obstacle");
    for (auto& [x, y] : result.path) {
        assert(!map.isOccupied(x, y) && "Path should not go through obstacles");
    }
    std::cout << "[PASS] testPathAroundObstacle" << std::endl;
}

void testNodesExplored() {
    GridMap map(10, 10);
    AStar astar(map);
    auto result = astar.findPath(0, 0, 9, 9);
    assert(result.nodesExplored > 0 && "Should explore at least one node");
    assert(result.nodesExplored <= 100 && "Should not explore all nodes");
    std::cout << "[PASS] testNodesExplored" << std::endl;
}

int main() {
    std::cout << "Running A* Tests..." << std::endl;
    testBasicPath();
    testBlockedPath();
    testStartEqualsGoal();
    testPathAroundObstacle();
    testNodesExplored();
    std::cout << "All A* tests passed!" << std::endl;
    return 0;
}
