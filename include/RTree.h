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
typedef bgi::rtree< Value, bgi::quadratic<16> > Rtree;

namespace rt
{
    /**
     * @param component
     * @return Kümenin çevreleyici en küçük dikdörtgenini döndürür.
     */
    Rect getBoundingBox(const Component& component)
    {

    }

    /**
     * @param boundingBoxes
     * @return
     */
    Rtree createRtree(const std::vector<Rect>& boundingBoxes)
    {
        return Rtree();
    }

    /**
     * @param indexes
     * @param rect
     * @param rtree
     * @param compIndex Eşleşme aranan kümenin indexi, kümenin kendisi index listesine kaydedilmemeli
     */
    void getIntersectingRects(std::vector<unsigned>& indexes, const Rect& rect, const Rtree& rtree, unsigned compIndex)
    {

    }
}

#endif //NARROWPASSAGEREMAKE_RTREE_H
