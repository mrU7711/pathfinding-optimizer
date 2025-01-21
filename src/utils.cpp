#include "utils.h"
#include <algorithm>

double euclideanDistance(int x1, int y1, int x2, int y2) {
    double dx = static_cast<double>(x2 - x1);
    double dy = static_cast<double>(y2 - y1);
    return std::sqrt(dx * dx + dy * dy);
}

double manhattanDistance(int x1, int y1, int x2, int y2) {
    return std::abs(x2 - x1) + std::abs(y2 - y1);
}

double chebyshevDistance(int x1, int y1, int x2, int y2) {
    return std::max(std::abs(x2 - x1), std::abs(y2 - y1));
}

std::vector<std::pair<int,int>> smoothPath(
    const std::vector<std::pair<int,int>>& path, int iterations) {
    if (path.size() <= 2) return path;

    std::vector<std::pair<double,double>> smooth;
    for (auto& p : path) {
        smooth.emplace_back(static_cast<double>(p.first),
                           static_cast<double>(p.second));
    }

    double weight_data = 0.5;
    double weight_smooth = 0.1;

    for (int iter = 0; iter < iterations; ++iter) {
        for (size_t i = 1; i < smooth.size() - 1; ++i) {
            double ox = smooth[i].first;
            double oy = smooth[i].second;
            smooth[i].first += weight_data * (path[i].first - smooth[i].first);
            smooth[i].first += weight_smooth * (smooth[i-1].first + smooth[i+1].first - 2.0 * smooth[i].first);
            smooth[i].second += weight_data * (path[i].second - smooth[i].second);
            smooth[i].second += weight_smooth * (smooth[i-1].second + smooth[i+1].second - 2.0 * smooth[i].second);
        }
    }

    std::vector<std::pair<int,int>> result;
    for (auto& p : smooth) {
        result.emplace_back(static_cast<int>(std::round(p.first)),
                           static_cast<int>(std::round(p.second)));
    }
    return result;
}

double pathLength(const std::vector<std::pair<int,int>>& path) {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += euclideanDistance(path[i-1].first, path[i-1].second,
                                    path[i].first, path[i].second);
    }
    return length;
}
