#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "grid_map.h"
#include <vector>
#include <utility>

struct DijkstraResult {
    std::vector<std::pair<int,int>> path;
    int nodesExplored;
    double pathCost;
    bool found;
};

class Dijkstra {
public:
    Dijkstra(const GridMap& map);
    DijkstraResult findPath(int startX, int startY, int goalX, int goalY);

private:
    const GridMap& map_;
};

#endif // DIJKSTRA_H
