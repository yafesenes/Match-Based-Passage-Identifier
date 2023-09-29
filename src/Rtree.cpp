#include "RTree.h"

Rect rt::getBoundingBox(const Component& component)
{
    Point p1(0, 0);
    Point p2(0, 0);
    return {p1, p2};
}

Rtree rt::createRtree(const std::vector<Rect>& boundingBoxes)
{
    return Rtree();
}

void rt::getIntersectingRects(std::vector<unsigned>& indexes, const Rect& rect, const Rtree& rtree, unsigned compIndex)
{

}