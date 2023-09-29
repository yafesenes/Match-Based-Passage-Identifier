#include <iostream>
#include "Types.h"
#include "Renderer.h"
#include "Image.h"
#include "NarrowFinder.h"

Map inflateMap(Map& map)
{
    Map newMap = map;
    size_t rows = map.size();
    size_t cols = map[0].size();
    int inflation_radius = 2;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (map[i][j] == 1) { // Obstacle detected
                for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                        int new_x = i + dx;
                        int new_y = j + dy;
                        if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols) {
                            newMap[new_x][new_y] = 1; // Inflate the obstacle
                        }
                    }
                }
            }
        }
    }
    return newMap;
}

Map loadMap()
{
    Image Img("res/Map2/willow.png");
    Map map = Img.getData();

    Map newMap = inflateMap(map);

    return newMap;
}


int main() {
    Map map = loadMap();

    NarrowFinder narrowFinder(map, 10);
    narrowFinder.calculatePassageValues();

    Renderer renderer = Renderer::instance();
    renderer.setMap(map);
    renderer.run();

    return 0;
}
