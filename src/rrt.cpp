#include "rrt.h"
#include "utils.h"
#include <cstdlib>
#include <ctime>
#include <algorithm>

RRT::RRT(const GridMap& map, int maxIterations, double stepSize)
    : map_(map), maxIterations_(maxIterations), stepSize_(stepSize),
      goalThreshold_(2.0) {
    std::srand(static_cast<unsigned>(std::time(nullptr)));
}

std::pair<int,int> RRT::randomSample() const {
    int x = std::rand() % map_.getWidth();
    int y = std::rand() % map_.getHeight();
    return {x, y};
}

int RRT::nearestNode(const std::vector<std::pair<int,int>>& tree,
                      int x, int y) const {
    int bestIdx = 0;
    double bestDist = euclideanDistance(tree[0].first, tree[0].second, x, y);
    for (size_t i = 1; i < tree.size(); ++i) {
        double d = euclideanDistance(tree[i].first, tree[i].second, x, y);
        if (d < bestDist) {
            bestDist = d;
            bestIdx = static_cast<int>(i);
        }
    }
    return bestIdx;
}

std::pair<int,int> RRT::steer(int fromX, int fromY, int toX, int toY) const {
    double dist = euclideanDistance(fromX, fromY, toX, toY);
    if (dist <= stepSize_) return {toX, toY};

    double ratio = stepSize_ / dist;
    int newX = fromX + static_cast<int>(ratio * (toX - fromX));
    int newY = fromY + static_cast<int>(ratio * (toY - fromY));
    return {newX, newY};
}

bool RRT::collisionFree(int x1, int y1, int x2, int y2) const {
    int steps = static_cast<int>(euclideanDistance(x1, y1, x2, y2)) + 1;
    for (int i = 0; i <= steps; ++i) {
        double t = (steps == 0) ? 0.0 : static_cast<double>(i) / steps;
        int cx = x1 + static_cast<int>(t * (x2 - x1));
        int cy = y1 + static_cast<int>(t * (y2 - y1));
        if (map_.isOccupied(cx, cy)) return false;
    }
    return true;
}

RRTResult RRT::findPath(int startX, int startY, int goalX, int goalY) {
    RRTResult result;
    result.found = false;
    result.nodesExplored = 0;

    std::vector<std::pair<int,int>> tree;
    std::vector<int> parent;
    tree.push_back({startX, startY});
    parent.push_back(-1);

    for (int i = 0; i < maxIterations_; ++i) {
        auto [rx, ry] = (i % 10 == 0) ? std::make_pair(goalX, goalY) : randomSample();

        int nearIdx = nearestNode(tree, rx, ry);
        auto [nx, ny] = steer(tree[nearIdx].first, tree[nearIdx].second, rx, ry);

        if (!map_.isValid(nx, ny) || map_.isOccupied(nx, ny)) continue;
        if (!collisionFree(tree[nearIdx].first, tree[nearIdx].second, nx, ny)) continue;

        tree.push_back({nx, ny});
        parent.push_back(nearIdx);
        result.nodesExplored++;

        if (euclideanDistance(nx, ny, goalX, goalY) <= goalThreshold_) {
            result.found = true;
            int idx = static_cast<int>(tree.size()) - 1;
            while (idx != -1) {
                result.path.push_back(tree[idx]);
                idx = parent[idx];
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }
    }

    return result;
}
