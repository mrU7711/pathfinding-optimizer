#include <iostream>
#include <string>
#include <cstring>
#include "grid_map.h"
#include "astar.h"
#include "dijkstra.h"
#include "rrt.h"
#include "utils.h"

void printUsage() {
    std::cout << "Usage: pathfinder --algorithm <astar|dijkstra|rrt> "
              << "--start x,y --goal x,y --map <config_file>" << std::endl;
}

std::pair<int,int> parseCoord(const std::string& s) {
    size_t comma = s.find(',');
    int x = std::stoi(s.substr(0, comma));
    int y = std::stoi(s.substr(comma + 1));
    return {x, y};
}

int main(int argc, char* argv[]) {
    std::string algorithm = "astar";
    std::string mapFile = "config/map_config.txt";
    int startX = 0, startY = 0;
    int goalX = 19, goalY = 14;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--algorithm") == 0 && i + 1 < argc) {
            algorithm = argv[++i];
        } else if (std::strcmp(argv[i], "--start") == 0 && i + 1 < argc) {
            auto [x, y] = parseCoord(argv[++i]);
            startX = x; startY = y;
        } else if (std::strcmp(argv[i], "--goal") == 0 && i + 1 < argc) {
            auto [x, y] = parseCoord(argv[++i]);
            goalX = x; goalY = y;
        } else if (std::strcmp(argv[i], "--map") == 0 && i + 1 < argc) {
            mapFile = argv[++i];
        }
    }

    GridMap map(20, 15);
    if (!map.loadFromFile(mapFile)) {
        std::cerr << "Failed to load map." << std::endl;
        return 1;
    }

    std::cout << "Map loaded: " << map.getWidth() << "x" << map.getHeight() << std::endl;
    std::cout << "Start: (" << startX << "," << startY << ") "
              << "Goal: (" << goalX << "," << goalY << ")" << std::endl;
    std::cout << "Algorithm: " << algorithm << std::endl;
    std::cout << std::string(40, '-') << std::endl;

    if (algorithm == "astar") {
        AStar astar(map);
        auto result = astar.findPath(startX, startY, goalX, goalY);
        if (result.found) {
            std::cout << "Path found! Length: " << pathLength(result.path)
                      << " Nodes explored: " << result.nodesExplored << std::endl;
            map.printMap(result.path);
        } else {
            std::cout << "No path found." << std::endl;
        }
    } else if (algorithm == "dijkstra") {
        Dijkstra dijkstra(map);
        auto result = dijkstra.findPath(startX, startY, goalX, goalY);
        if (result.found) {
            std::cout << "Path found! Length: " << pathLength(result.path)
                      << " Nodes explored: " << result.nodesExplored << std::endl;
            map.printMap(result.path);
        } else {
            std::cout << "No path found." << std::endl;
        }
    } else if (algorithm == "rrt") {
        RRT rrt(map);
        auto result = rrt.findPath(startX, startY, goalX, goalY);
        if (result.found) {
            std::cout << "Path found! Nodes explored: " << result.nodesExplored << std::endl;
            map.printMap(result.path);
        } else {
            std::cout << "No path found after max iterations." << std::endl;
        }
    } else {
        std::cerr << "Unknown algorithm: " << algorithm << std::endl;
        printUsage();
        return 1;
    }

    return 0;
}
