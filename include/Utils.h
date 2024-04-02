#ifndef NARROWPASSAGEREMAKE_UTILS_H
#define NARROWPASSAGEREMAKE_UTILS_H

#include <optional>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "Types.h"
#include <thread>
#include <mutex>

namespace utils {

    /**
     * @param p1
     * @param p2
     * @return returns distance between given points
     */
    double getDistance(const Point& p1, const Point& p2);

    /** @brief clusters neighbor cells
     * @param connectedComponents vector to fill
     * @param map
     * @param componentValue type of the cluster (0: free space, 1: engel)
     * @param dir 8-dir or 4-dir
     */
    void getConnectedComponents(std::vector<Component> &connectedComponents,
                                  const Map& map,
                                  bool componentValue = 1,
                                  int dir = 8,
                                  Point refPoint = Point(0,0));

    std::vector<Point> bresenham(Point p0, Point p1);

    std::vector<Component> tearComponent(const Component& c);

    Rect ComponentToRect(const std::vector<Point> &Component);

    Convex ConvexHull(Component P);

    bool compare(Point p1, Point p2);

    int cross(const Point &O, const Point &A, const Point &B);

    Component borderizeComponent(const Component& component, const Map& occupancyMap, bool componentValue);

    bool isInsideConvex(const std::vector<Point> &convexHull, const Point &point);

    void ClusterConvexPolygon(const std::vector<Point> &polygon, const std::vector<Point> &points, std::pair<std::vector<Point>, std::vector<Point>>& PolygonPoints);

}

#endif //NARROWPASSAGEREMAKE_UTILS_H
