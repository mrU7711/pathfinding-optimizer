#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <vector>
#include <string>
#include <utility>

struct Cell {
    int x;
    int y;
    bool occupied;
};

class GridMap {
public:
    GridMap(int width, int height);

    void setObstacle(int x, int y);
    void clearObstacle(int x, int y);
    bool isOccupied(int x, int y) const;
    bool isValid(int x, int y) const;

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

    bool loadFromFile(const std::string& filepath);
    void printMap(const std::vector<std::pair<int,int>>& path = {}) const;

    std::vector<std::pair<int,int>> getNeighbors(int x, int y, bool allowDiagonal = true) const;

private:
    int width_;
    int height_;
    std::vector<std::vector<bool>> grid_;
};

#endif // GRID_MAP_H
