#include "NarrowFinder.h"

NarrowFinder::NarrowFinder(Map map, float threshold) : _map(std::move(map)), _threshold(threshold)
{
    _passageValues.resize(map.size(), std::vector<float>(map[0].size(), std::min(map.size(), map[0].size())));
}

void NarrowFinder::calculatePassageValues()
{
    std::vector<Component> connectedComponents;
    getConnectedComponents(connectedComponents);

    foreignMatcher(connectedComponents);

    for (const Component& c : connectedComponents)
        ownMatcher(c);
}




void NarrowFinder::foreignMatcher(const std::vector<Component> &components)
{
    // İçi boş kümelerin elde edilmesi
    std::vector<Component> borderizedComponents;
    borderizedComponents.reserve(components.size());
    for (const auto& c : components)
    {
        // TODO: debug için, emin olunduğu takdirde silinecek
        if (c.empty())
            throw std::runtime_error("[foreignMatcher] Empty component");
        borderizedComponents.push_back(utils::borderizeComponent(c));
    }

    // Kümelerin bounding box'larının elde edilmesi
    std::vector<Rect> boundingBoxes;
    boundingBoxes.reserve(components.size());
    for (const auto& c : borderizedComponents)
        boundingBoxes.push_back(rt::getBoundingBox(c));

    // Rtree oluşturulması
    Rtree rtree = rt::createRtree(boundingBoxes);

    std::vector<unsigned> indexes;
    double minDist = std::numeric_limits<double>::max();
    std::optional<Point> minDistPoint;
    // Her küme için
    for (int i = 0; i < boundingBoxes.size(); i++)
    {
        rt::getIntersectingRects(indexes, boundingBoxes[i], rtree, i);
        // Kümedeki her eleman için
        for (const auto& j : borderizedComponents[i])
        {
            // Eşleşen kümelerin her elemanı için
            for (unsigned index : indexes)
            {
                for (const auto& k : borderizedComponents[index])
                {
                    double dist = utils::getDistance(j, k);
                    if (dist < _threshold && dist < minDist)
                    {
                        minDist = dist;
                        minDistPoint = k;
                    }
                }
            }
            if (minDistPoint.has_value())
            {
                collisionCheck(j, minDistPoint.value());
            }
            minDist = std::numeric_limits<double>::max();
            minDistPoint.reset();
        }

        indexes.clear();
    }
}

void NarrowFinder::foreignMatcherRect(const std::vector<Component> &components)
{
    // İçi boş kümelerin elde edilmesi
    std::vector<Component> borderizedComponents;
    borderizedComponents.reserve(components.size());
    for (const auto& c : components)
    {
        // TODO: debug için, emin olunduğu takdirde silinecek
        if (c.empty())
            throw std::runtime_error("[foreignMatcher] Empty component");
        borderizedComponents.push_back(utils::borderizeComponent(c));
    }

    for (const Component& c : borderizedComponents)
    {
        for (const Point& p : c)
        {
            cv::Mat clusteredMap = clusterMap(Point(p.x - _threshold, p.y - _threshold),
                                              Point(p.x + _threshold, p.y + _threshold));

        }
    }
}

cv::Mat NarrowFinder::clusterMap(Point min, Point max) {
    return cv::Mat();
}

void NarrowFinder::ownMatcher(const Component &component)
{

}

void NarrowFinder::collisionCheck(const Point& p1, const Point& p2)
{

}
