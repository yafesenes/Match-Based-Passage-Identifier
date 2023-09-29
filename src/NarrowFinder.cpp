#include "NarrowFinder.h"

NarrowFinder::NarrowFinder(Map map, float threshold) : _map(std::move(map)), _threshold(threshold)
{
    _passageValues.resize(_map.size(), std::vector<float>(_map[0].size(), std::min(_map.size(), _map[0].size())));
}

void NarrowFinder::calculatePassageValues()
{
    std::vector<Component> borderizedComponents = utils::borderizeComponents(this->_map);


    foreignMatcher(borderizedComponents);

    std::vector<Component> connectedComponents;
    utils::getConnectedComponents(connectedComponents, this->_map);

    for (const Component& c : connectedComponents)
        ownMatcher(c);
}

void NarrowFinder::foreignMatcher(const std::vector<Component> &components)
{
    // Kümelerin bounding box'larının elde edilmesi
    std::vector<Rect> boundingBoxes;
    boundingBoxes.reserve(components.size());
    for (const auto& c : components)
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
        for (const auto& j : components[i])
        {
            // Eşleşen kümelerin her elemanı için
            for (unsigned index : indexes)
            {
                for (const auto& k : components[index])
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

void NarrowFinder::ownMatcher(const Component &component)
{

}

void NarrowFinder::collisionCheck(const Point& p1, const Point& p2)
{

}
