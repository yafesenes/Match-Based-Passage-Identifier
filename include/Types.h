#ifndef NARROWPASSAGEREMAKE_TYPES_H
#define NARROWPASSAGEREMAKE_TYPES_H

#include <vector>

struct Point
{
    int x, y;
    Point(int x, int y) : x(x), y(y) {}
};

typedef std::vector<std::vector<bool>> Map;
typedef std::pair<Point, Point> Rect;
typedef std::vector<Point> Component;
typedef std::vector<Point> Convex;
typedef std::vector<std::vector<float>> PassageValues;

#endif //NARROWPASSAGEREMAKE_TYPES_H
