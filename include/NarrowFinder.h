#ifndef NARROWPASSAGEREMAKE_NARROWFINDER_H
#define NARROWPASSAGEREMAKE_NARROWFINDER_H

#include <vector>
#include <memory>
#include <optional>
#include <utility>
#include <limits>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include "Types.h"
#include "Utils.h"
#include "Renderer.h"
#include "RTree.h"
#include "TicToc.h"

class NarrowFinder {
public:
    NarrowFinder(Map map, float threshold);

    void calculatePassageValues();

private:
    /**
     * @brief İçine borderize edilmiş component almalı.
     * @param components Obstacle kümeleri
     */
    void foreignMatcher(const std::vector<Component>& components);

    /**
     * @param component Obstacle kümesi
     */
    void ownMatcher(const Component& component);

    /**
     * @brief Arasında engel bulunmayan eşleşmeleri kullanarak PassageValues hesaplar
     */
    void collisionCheck(const Point& p1, const Point& p2);

    /**
     * @param component
     * @return İçi boşaltılmış küme döndürür
     */
    Component borderizeComponent(const Component& component);

private:
    const int _threshold;
    const Map _map;
    PassageValues _passageValues;
};



#endif //NARROWPASSAGEREMAKE_NARROWFINDER_H
