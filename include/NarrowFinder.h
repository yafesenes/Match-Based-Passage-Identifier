#ifndef NARROWPASSAGEREMAKE_NARROWFINDER_H
#define NARROWPASSAGEREMAKE_NARROWFINDER_H

#include <vector>
#include <memory>
#include <optional>
#include <utility>
#include <limits>
#include <opencv2/opencv.hpp>
#include "Types.h"
#include "Utils.h"
#include "RTree.h"

class NarrowFinder {
public:
    NarrowFinder(Map map, float threshold);

    void calculatePassageValues();

private:

    void getConnectedComponents(std::vector<Component> &connectedComponents,
                                bool componentValue = 1,
                                int dir = 8,
                                std::optional<std::pair<Point, Point>> mapBorder = std::nullopt);

    /**
     * @param components Obstacle kümeleri
     */
    void foreignMatcher(const std::vector<Component>& components);
    void foreignMatcherRect(const std::vector<Component>& components);
    cv::Mat clusterMap(Point min, Point max);

    /**
     * @param component Obstacle kümesi
     */
    void ownMatcher(const Component& component);

    /**
     * @brief Arasında engel bulunmayan eşleşmeleri kullanarak PassageValues hesaplar
     */
    void collisionCheck(const Point& p1, const Point& p2);

private:
    const float _threshold;

    const Map _map;
    PassageValues _passageValues;
};



#endif //NARROWPASSAGEREMAKE_NARROWFINDER_H
