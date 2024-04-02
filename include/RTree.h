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
     * @param component
     * @return Kümenin çevreleyici en küçük dikdörtgenini döndürür.
     */
    Rect getBoundingBox(const Component &component);

    /**
     * @param boundingBoxes Componentlerin dikdörtgen hali
     * @return
     */
    Rtree createRtree(const std::vector<Rect> &boundingBoxes);

    /**
     * @brief Her obstacle biriminin eşleşeceği engelleri belirlemek için uygulanan ilk filtredir
     * @param indexes İçi doldurulacak olan değişken
     * @param p Etrafında querybox oluşturulacak olan nokta
     * @param rtree Kesişmeleri öngörmek için kullanılan veri yapısı
     * @param compIndex Eşleşme aranan kümenin indexi, kümenin kendisi index listesine kaydedilmemeli
     * @param thresholdValue Querybox'ın büyüklüğünün ayarlanmasını sağlayan piksel sayısı
     */
    void getIntersectingRects(std::vector<unsigned> &indexes, const Point &p, const Rtree &rtree, unsigned compIndex,
                              int thresholdValue);
}

#endif //NARROWPASSAGEREMAKE_RTREE_H
