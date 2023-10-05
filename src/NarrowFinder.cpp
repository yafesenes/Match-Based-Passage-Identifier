#include "NarrowFinder.h"

NarrowFinder::NarrowFinder(Map map, float threshold) : _map(std::move(map)), _threshold(std::min(map.size(), map[0].size()) * threshold)
{
    _passageValues.resize(_map.size(), std::vector<float>(_map[0].size(), std::min(_map.size(), _map[0].size())));
}

void NarrowFinder::calculatePassageValues()
{
    tic("calculatePassageValues");
    std::vector<Component> connectedComponents;
    utils::getConnectedComponents(connectedComponents, this->_map);

    foreignMatcher(connectedComponents);
    Renderer::instance().drawMatches(_passageValues);

    for (const Component& c : connectedComponents)
        ownMatcher(c);

    toc("calculatePassageValues");
}

void NarrowFinder::foreignMatcher(const std::vector<Component> &components)
{
    tic("foreignMatcher");
    std::vector<Component> borderizedComponents;

    for (const auto& c : components)
        borderizedComponents.push_back(borderizeComponent(c));

    // Kümelerin bounding box'larının elde edilmesi
    std::vector<Rect> boundingBoxes;
    boundingBoxes.reserve(borderizedComponents.size());
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
        // Kümedeki her eleman için
        for (const auto& j : borderizedComponents[i])
        {
            rt::getIntersectingRects(indexes, j, rtree, i, _threshold);

            pair<Point, Point> thRect = {Point(j.x - _threshold,
                                               j.y - _threshold),
                                         Point(j.x + _threshold,
                                               j.y + _threshold)};
            // Eşleşen kümelerin her elemanı için
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
    toc("foreignMatcher");
}

void NarrowFinder::ownMatcher(const Component &component)
{
    tic("ownMatcher");
    std::vector<Component> components = utils::tearComponent(component);
    foreignMatcher(components);

    for (const auto& comp : components)
    {
        ownMatcher(comp);
    }

    toc("ownMatcher");
}

void NarrowFinder::collisionCheck(const Point& p1, const Point& p2)
{
    tic("collisionCheck");
    vector<Point> midPoints = utils::bresenham(p1, p2);
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
    toc("collisionCheck");
}

Component NarrowFinder::borderizeComponent(const Component& component)
{
    tic("borderizeComponent");
    std::vector<int> directionsX = {0, 1, 0, -1, 1, -1, 1, -1};
    std::vector<int> directionsY = {1, 0, -1, 0, -1, 1, 1, -1};
    std::vector<Point> newComponent;
    std::mutex mtx;

    auto processComponent = [&](int start, int end) {
        std::vector<Point> localComponent;
        for (int i = start; i < end; ++i)
        {
            auto& j = component[i];
            for (size_t k = 0; k < directionsX.size(); k++)
            {
                int x = j.x + directionsX[k];
                int y = j.y + directionsY[k];

                if (y >= _map.size() || x >= _map[y].size() || x < 0 || y < 0)
                    continue;

                if (_map[y][x] == 0)
                {
                    localComponent.push_back(j);
                    break;
                }
            }
        }
        std::lock_guard<std::mutex> lock(mtx);
        newComponent.insert(newComponent.end(), localComponent.begin(), localComponent.end());
    };

    int numThreads = std::thread::hardware_concurrency()/2;
    int blockSize = component.size() / numThreads;
    std::vector<std::thread> threads;

    for (int i = 0; i < numThreads; ++i)
    {
        int start = i * blockSize;
        int end = (i == numThreads - 1) ? component.size() : start + blockSize;
        threads.emplace_back(processComponent, start, end);
    }

    for (auto& thread : threads)
    {
        if (thread.joinable())
            thread.join();
    }
    toc("borderizeComponent");
    return newComponent;
}

