#include <iostream>
#include "RTree.h"


Rect rt::getBoundingBox(const Component &component)
{
    tic("rt::getBoundingBox");
    Point TopLeft = component[0];
    Point BotRight = component[0];

    for (const Point &p:component)
    {
        if (p.x<TopLeft.x)
            TopLeft.x = p.x;

        if (p.y<TopLeft.y)
            TopLeft.y = p.y;

        if (p.x>BotRight.x)
            BotRight.x = p.x;

        if (p.y>BotRight.y)
            BotRight.y = p.y;
    }

    toc("rt::getBoundingBox");
    return {TopLeft, BotRight};
}

Rtree rt::createRtree(const std::vector<Rect> &boundingBoxes)
{
    tic("rt::createRtree");
    Rtree rtree;
    int i = 0;

    for (const auto &rect: boundingBoxes) {
        Box rectBox(RPoint(rect.first.x, rect.first.y),
                    RPoint(rect.second.x, rect.second.y));
        rtree.insert(std::make_pair(rectBox, i++));
    }

    toc("rt::createRtree");
    return rtree;
}

void rt::getIntersectingRects(std::vector<unsigned> &indexes, const Point &p, const Rtree &rtree, unsigned compIndex,
                              int thresholdValue)
{
    tic("rt::getIntersectingRects");
    indexes.clear();
    Box queryBox(RPoint(p.x - thresholdValue,
                        p.y - thresholdValue),
                 RPoint(p.x + thresholdValue,
                        p.y + thresholdValue));

    std::vector<Value> result;
    rtree.query(bgi::intersects(queryBox), std::back_inserter(result));

    if (result.size() == 1)
    {
        toc("rt::getIntersectingRects");
        return;
    }

    indexes.resize(result.size()-1);
    size_t j = 0;
    for (int i = 0; i < result.size(); i++)
    {
        if (result[i].second == compIndex)
            continue;

        // cout << "j: " << j << " result: " << i.second << " comp_index: " << componentIndex << " result_size: " << result.size() << endl;
        indexes[j++] = result[i].second;
    }
    toc("rt::getIntersectingRects");
}