#ifndef NARROWPASSAGEREMAKE_RTREE_H
#define NARROWPASSAGEREMAKE_RTREE_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "Types.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<int, 2, bg::cs::cartesian> RPoint;
typedef bg::model::box<RPoint> Box;
typedef std::pair<Box, unsigned> Value;
typedef bgi::rtree<Value, bgi::quadratic<16> > Rtree;

namespace rt {
    /**
     * @return Returns the smallest enclosing rectangle of the component.
     */
    Rect getBoundingBox(const Component &component);

    Rtree createRtree(const std::vector<Rect> &boundingBoxes);

    /**
     * @brief The first filter applied to determine the obstacles that each obstacle unit will match with
     * @param indexes The variable to be filled
     * @param p The point around which the querybox will be created
     * @param rtree The data structure used for predicting intersections
     * @param compIndex The index of the component being searched for matches, the component itself should not be recorded in the index list
     * @param thresholdValue The number of pixels that adjusts the size of the querybox
     */
    void getIntersectingRects(std::vector<unsigned> &indexes, const Point &p, const Rtree &rtree, unsigned compIndex,
                              int thresholdValue);
}

#endif //NARROWPASSAGEREMAKE_RTREE_H
