#include "dijkstra.h"
#include "utils.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <limits>

struct DNode {
    int x, y;
    double dist;
    bool operator>(const DNode& other) const { return dist > other.dist; }
};

static long long dkey(int x, int y) {
    return static_cast<long long>(x) * 100000 + y;
}

Dijkstra::Dijkstra(const GridMap& map) : map_(map) {}

DijkstraResult Dijkstra::findPath(int startX, int startY, int goalX, int goalY) {
    DijkstraResult result;
    result.found = false;
    result.nodesExplored = 0;
    result.pathCost = 0.0;

    if (!map_.isValid(startX, startY) || !map_.isValid(goalX, goalY)) {
        return result;
    }
    if (map_.isOccupied(startX, startY) || map_.isOccupied(goalX, goalY)) {
        return result;
    }

    std::priority_queue<DNode, std::vector<DNode>, std::greater<DNode>> pq;
    std::unordered_map<long long, double> dist;
    std::unordered_map<long long, long long> prev;

    long long startKey = dkey(startX, startY);
    dist[startKey] = 0.0;
    pq.push({startX, startY, 0.0});

    while (!pq.empty()) {
        DNode current = pq.top();
        pq.pop();
        result.nodesExplored++;

        long long ck = dkey(current.x, current.y);
        if (current.dist > dist[ck]) continue;

        if (current.x == goalX && current.y == goalY) {
            result.found = true;
            result.pathCost = current.dist;
            long long tk = ck;
            while (prev.count(tk)) {
                int px = static_cast<int>(tk / 100000);
                int py = static_cast<int>(tk % 100000);
                result.path.emplace_back(px, py);
                tk = prev[tk];
            }
            result.path.emplace_back(startX, startY);
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        for (auto& [nx, ny] : map_.getNeighbors(current.x, current.y)) {
            double moveCost = euclideanDistance(current.x, current.y, nx, ny);
            double newDist = current.dist + moveCost;
            long long nk = dkey(nx, ny);

            if (!dist.count(nk) || newDist < dist[nk]) {
                dist[nk] = newDist;
                prev[nk] = ck;
                pq.push({nx, ny, newDist});
            }
        }
    }

    return result;
}
