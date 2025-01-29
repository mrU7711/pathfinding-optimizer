#include <iostream>
#include <cassert>
#include "../src/grid_map.h"
#include "../src/dijkstra.h"
#include "../src/utils.h"

void testBasicPath() {
    GridMap map(10, 10);
    Dijkstra dijkstra(map);
    auto result = dijkstra.findPath(0, 0, 9, 9);
    assert(result.found && "Dijkstra should find path in empty grid");
    assert(result.path.front() == std::make_pair(0, 0));
    assert(result.path.back() == std::make_pair(9, 9));
    std::cout << "[PASS] testBasicPath" << std::endl;
}

void testBlockedPath() {
    GridMap map(5, 5);
    for (int y = 0; y < 5; ++y) {
        map.setObstacle(2, y);
    }
    Dijkstra dijkstra(map);
    auto result = dijkstra.findPath(0, 0, 4, 4);
    assert(!result.found && "Dijkstra should not find path through wall");
    std::cout << "[PASS] testBlockedPath" << std::endl;
}

void testOptimalPath() {
    GridMap map(5, 1);
    Dijkstra dijkstra(map);
    auto result = dijkstra.findPath(0, 0, 4, 0);
    assert(result.found);
    assert(result.pathCost <= 5.0 && "Path cost should be optimal");
    std::cout << "[PASS] testOptimalPath" << std::endl;
}

void testInvalidStart() {
    GridMap map(10, 10);
    map.setObstacle(0, 0);
    Dijkstra dijkstra(map);
    auto result = dijkstra.findPath(0, 0, 9, 9);
    assert(!result.found && "Should fail with obstacle at start");
    std::cout << "[PASS] testInvalidStart" << std::endl;
}

int main() {
    std::cout << "Running Dijkstra Tests..." << std::endl;
    testBasicPath();
    testBlockedPath();
    testOptimalPath();
    testInvalidStart();
    std::cout << "All Dijkstra tests passed!" << std::endl;
    return 0;
}
