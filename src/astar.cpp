#include "astar.h"
#include "utils.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <functional>

struct Node {
    int x, y;
    double g, f;
    bool operator>(const Node& other) const { return f > other.f; }
};

static long long key(int x, int y) {
    return static_cast<long long>(x) * 100000 + y;
}

AStar::AStar(const GridMap& map) : map_(map) {}

double AStar::heuristic(int x1, int y1, int x2, int y2) const {
    return manhattanDistance(x1, y1, x2, y2);
}

AStarResult AStar::findPath(int startX, int startY, int goalX, int goalY) {
    AStarResult result;
    result.found = false;
    result.nodesExplored = 0;
    result.pathCost = 0.0;

    if (!map_.isValid(startX, startY) || !map_.isValid(goalX, goalY)) {
        return result;
    }
    if (map_.isOccupied(startX, startY) || map_.isOccupied(goalX, goalY)) {
        return result;
    }

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_map<long long, double> gScore;
    std::unordered_map<long long, long long> cameFrom;

    long long startKey = key(startX, startY);
    gScore[startKey] = 0.0;
    openSet.push({startX, startY, 0.0, heuristic(startX, startY, goalX, goalY)});

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();
        result.nodesExplored++;

        if (current.x == goalX && current.y == goalY) {
            result.found = true;
            result.pathCost = current.g;
            // Reconstruct path
            long long ck = key(current.x, current.y);
            while (cameFrom.count(ck)) {
                int px = static_cast<int>(ck / 100000);
                int py = static_cast<int>(ck % 100000);
                result.path.emplace_back(px, py);
                ck = cameFrom[ck];
            }
            result.path.emplace_back(startX, startY);
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        long long currentKey = key(current.x, current.y);
        if (current.g > gScore[currentKey]) continue;

        for (auto& [nx, ny] : map_.getNeighbors(current.x, current.y)) {
            double moveCost = euclideanDistance(current.x, current.y, nx, ny);
            double tentativeG = current.g + moveCost;
            long long nk = key(nx, ny);

            if (!gScore.count(nk) || tentativeG < gScore[nk]) {
                gScore[nk] = tentativeG;
                cameFrom[nk] = currentKey;
                double f = tentativeG + heuristic(nx, ny, goalX, goalY);
                openSet.push({nx, ny, tentativeG, f});
            }
        }
    }

    return result;
}
