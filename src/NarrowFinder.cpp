#include "NarrowFinder.h"

NarrowFinder::NarrowFinder(Map map, float threshold) : _map(std::move(map)), _threshold(std::min(map.size(), map[0].size()) * threshold)
{
    _passageValues.resize(_map.size(), std::vector<float>(_map[0].size(), std::min(_map.size(), _map[0].size())));
}

void NarrowFinder::calculatePassageValues()
{
    std::vector<Component> connectedComponents;
    utils::getConnectedComponents(connectedComponents, this->_map);

    foreignMatcher(connectedComponents);

    for (const Component& c : connectedComponents)
        ownMatcher(c);
}

void NarrowFinder::foreignMatcher(const std::vector<Component> &components)
{
    if (components.size() < 2)
    {
        return;
    }

    std::vector<Component> borderizedComponents;
    for (const auto& c : components)
        borderizedComponents.push_back(utils::borderizeComponent(c, _map, 1));

    std::vector<Rect> boundingBoxes;
    boundingBoxes.reserve(borderizedComponents.size());
    for (const auto& c : borderizedComponents)
        boundingBoxes.push_back(rt::getBoundingBox(c));

    Rtree rtree = rt::createRtree(boundingBoxes);

    std::vector<unsigned> indexes;
    double minDist = std::numeric_limits<double>::max();
    std::optional<Point> minDistPoint;
    for (int i = 0; i < boundingBoxes.size(); i++)
    {
        for (const auto& j : borderizedComponents[i])
        {
            rt::getIntersectingRects(indexes, j, rtree, i, _threshold);

            std::pair<Point, Point> thRect = {Point(j.x - _threshold,
                                               j.y - _threshold),
                                         Point(j.x + _threshold,
                                               j.y + _threshold)};
            for (unsigned index : indexes)
            {
                for (const auto& k : borderizedComponents[index])
                {
                    if ((k.x > thRect.first.x && k.x < thRect.second.x && k.y > thRect.first.y && k.y < thRect.second.y))
                    {
                        double dist = utils::getDistance(j, k);
                        if (dist < minDist)
                        {
                            minDist = dist;
                            minDistPoint = k;
                        }
                    }
                }
            }
            if (minDistPoint.has_value())
            {
                collisionCheck(j, minDistPoint.value());
            }
            minDist = std::numeric_limits<double>::max();
            minDistPoint.reset();
            indexes.clear();
        }
    }
}

void NarrowFinder::ownMatcher(const Component &component)
{
    if (component.size() < 3)
    {
        return;
    }

    Convex compConvex = utils::ConvexHull(component);

    Rect rectCoords = utils::ComponentToRect(compConvex);
    Map rectMap(rectCoords.second.y - rectCoords.first.y + 1, std::vector<bool>(rectCoords.second.x - rectCoords.first.x + 1, 0));

    for (const auto& p : component)
        rectMap[p.y - rectCoords.first.y][p.x - rectCoords.first.x] = 1;


    for (int i=0; i<compConvex.size()-1; i++)
    {
        std::vector<Point> midPoints = utils::bresenham(compConvex[i],compConvex[i+1]);

        for (const auto& p : midPoints)
            rectMap[p.y - rectCoords.first.y][p.x - rectCoords.first.x] = 1;
    }

    std::vector<Component> freeSpaceComponents;
    utils::getConnectedComponents(freeSpaceComponents, rectMap, 0, 4, rectCoords.first);

    for (const auto& freeSpaceComponent : freeSpaceComponents)
    {
        if (!utils::isInsideConvex(compConvex, freeSpaceComponent[0])) {
            continue;
        }
        if (freeSpaceComponent.size() < 3) {
            continue;
        }

        Component borderedFreeComp = utils::borderizeComponent(freeSpaceComponent, _map, 0);
        Convex freeConvex = utils::ConvexHull(borderedFreeComp);

        std::pair<std::vector<Point>, std::vector<Point>> PolygonPoints;
        utils::ClusterConvexPolygon(freeConvex, component, PolygonPoints);

        if (PolygonPoints.second.empty())
            continue;

        Map insidePointMap(rectCoords.second.y - rectCoords.first.y + 1, std::vector<bool>(rectCoords.second.x - rectCoords.first.x + 1, 0));
        for (const auto& p : PolygonPoints.second)
            insidePointMap[p.y - rectCoords.first.y][p.x - rectCoords.first.x] = 1;

        std::vector<Component> insideComponents;
        utils::getConnectedComponents(insideComponents, insidePointMap, 1, 8, rectCoords.first);

        if (!PolygonPoints.first.empty())
            foreignMatcher({PolygonPoints.first, PolygonPoints.second});

        foreignMatcher(insideComponents);

        for (const auto& comp : insideComponents)
            ownMatcher(comp);
    }
}

void NarrowFinder::collisionCheck(const Point& p1, const Point& p2)
{
    std::vector<Point> midPoints = utils::bresenham(p1, p2);
    bool flag = 0;
    for (auto & midPoint : midPoints)
    {
        if (_map[midPoint.y][midPoint.x] == 1)
        {
            flag = 1;
            break;
        }
    }

    if (!flag)
    {
        double dist = utils::getDistance(p1, p2);
        for (auto & midPoint : midPoints)
        {
            if (_passageValues[midPoint.y][midPoint.x] > dist)
            {
                _passageValues[midPoint.y][midPoint.x] = static_cast<float>(dist);
            }
        }
    }
}
