#ifndef ASTAR_H
#define ASTAR_H

#include "grid_map.h"
#include <vector>
#include <utility>

struct AStarResult {
    std::vector<std::pair<int,int>> path;
    int nodesExplored;
    double pathCost;
    bool found;
};

class AStar {
public:
    AStar(const GridMap& map);
    AStarResult findPath(int startX, int startY, int goalX, int goalY);

private:
    const GridMap& map_;
    double heuristic(int x1, int y1, int x2, int y2) const;
};

#endif // ASTAR_H
