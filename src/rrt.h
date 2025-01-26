#ifndef RRT_H
#define RRT_H

#include "grid_map.h"
#include <vector>
#include <utility>

struct RRTResult {
    std::vector<std::pair<int,int>> path;
    int nodesExplored;
    bool found;
};

class RRT {
public:
    RRT(const GridMap& map, int maxIterations = 5000, double stepSize = 1.5);
    RRTResult findPath(int startX, int startY, int goalX, int goalY);

private:
    const GridMap& map_;
    int maxIterations_;
    double stepSize_;
    double goalThreshold_;

    std::pair<int,int> randomSample() const;
    int nearestNode(const std::vector<std::pair<int,int>>& tree,
                    int x, int y) const;
    std::pair<int,int> steer(int fromX, int fromY, int toX, int toY) const;
    bool collisionFree(int x1, int y1, int x2, int y2) const;
};

#endif // RRT_H
