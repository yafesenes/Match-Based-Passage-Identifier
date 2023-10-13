#ifndef NARROWPASSAGEREMAKE_UTILS_H
#define NARROWPASSAGEREMAKE_UTILS_H

#include <optional>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "Types.h"
#include "TicToc.h"
#include <thread>
#include <mutex>

namespace utils {

    /**
     * @param p1
     * @param p2
     * @return İki nokta arasındaki mesafeyi döndürür.
     */
    double getDistance(const Point& p1, const Point& p2);

    /** @brief Birbirine komşuluğu bulunan harita elemanlarını gruplar
     * @param connectedComponents Sonuçlar içine yazılacak değişken
     * @param map Kümelenecek harita
     * @param componentValue Kümelenecek eleman türü (0: free space, 1: engel)
     * @param dir Komşuluk yön sayısı
     */
    void getConnectedComponents(std::vector<Component> &connectedComponents,
                                  const Map& map,
                                  bool componentValue = 1,
                                  int dir = 8,
                                  Point refPoint = Point(0,0));

    std::vector<Point> bresenham(Point p0, Point p1);

    std::vector<Component> tearComponent(const Component& c);

    Rect ComponentToRect(const vector<Point> &Component);

    Convex ConvexHull(Component P);

    bool compare(Point p1, Point p2);

    int cross(const Point &O, const Point &A, const Point &B);

    Component borderizeComponent(const Component& component, const Map& occupancyMap, bool componentValue);

    bool isInsideConvex(const std::vector<Point> &convexHull, const Point &point);

    void ClusterConvexPolygon(const vector<Point> &polygon, const vector<Point> &points, pair<vector<Point>, vector<Point>>& PolygonPoints);

}

#endif //NARROWPASSAGEREMAKE_UTILS_H
