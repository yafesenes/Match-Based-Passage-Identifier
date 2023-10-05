#ifndef NARROWPASSAGEREMAKE_UTILS_H
#define NARROWPASSAGEREMAKE_UTILS_H

#include <optional>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "Types.h"
#include "TicToc.h"

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
                                  int dir = 8);

    std::vector<Point> bresenham(Point p0, Point p1);

    std::vector<Component> tearComponent(const Component& c);
}

#endif //NARROWPASSAGEREMAKE_UTILS_H
