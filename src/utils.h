#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <utility>
#include <cmath>

double euclideanDistance(int x1, int y1, int x2, int y2);
double manhattanDistance(int x1, int y1, int x2, int y2);
double chebyshevDistance(int x1, int y1, int x2, int y2);

std::vector<std::pair<int,int>> smoothPath(
    const std::vector<std::pair<int,int>>& path, int iterations = 5);

double pathLength(const std::vector<std::pair<int,int>>& path);

#endif // UTILS_H
