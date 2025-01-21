#include "grid_map.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

GridMap::GridMap(int width, int height)
    : width_(width), height_(height),
      grid_(height, std::vector<bool>(width, false)) {}

void GridMap::setObstacle(int x, int y) {
    if (isValid(x, y)) {
        grid_[y][x] = true;
    }
}

void GridMap::clearObstacle(int x, int y) {
    if (isValid(x, y)) {
        grid_[y][x] = false;
    }
}

bool GridMap::isOccupied(int x, int y) const {
    if (!isValid(x, y)) return true;
    return grid_[y][x];
}

bool GridMap::isValid(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool GridMap::loadFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open map file: " << filepath << std::endl;
        return false;
    }

    std::string line;
    // Skip comment lines
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        int w, h;
        if (iss >> w >> h) {
            width_ = w;
            height_ = h;
            grid_.assign(height_, std::vector<bool>(width_, false));
            break;
        }
    }

    // Read obstacles
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        int ox, oy;
        if (iss >> ox >> oy) {
            setObstacle(ox, oy);
        }
    }

    return true;
}

void GridMap::printMap(const std::vector<std::pair<int,int>>& path) const {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            bool onPath = std::find(path.begin(), path.end(),
                                     std::make_pair(x, y)) != path.end();
            if (onPath) {
                std::cout << "* ";
            } else if (grid_[y][x]) {
                std::cout << "# ";
            } else {
                std::cout << ". ";
            }
        }
        std::cout << std::endl;
    }
}

std::vector<std::pair<int,int>> GridMap::getNeighbors(int x, int y, bool allowDiagonal) const {
    std::vector<std::pair<int,int>> neighbors;
    // 4-directional
    int dx4[] = {0, 0, 1, -1};
    int dy4[] = {1, -1, 0, 0};
    // 8-directional (additional diagonals)
    int dx8[] = {0, 0, 1, -1, 1, 1, -1, -1};
    int dy8[] = {1, -1, 0, 0, 1, -1, 1, -1};

    int count = allowDiagonal ? 8 : 4;
    int* dx = allowDiagonal ? dx8 : dx4;
    int* dy = allowDiagonal ? dy8 : dy4;

    for (int i = 0; i < count; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (isValid(nx, ny) && !isOccupied(nx, ny)) {
            neighbors.emplace_back(nx, ny);
        }
    }
    return neighbors;
}
