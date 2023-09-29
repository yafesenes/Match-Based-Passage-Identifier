#ifndef NARROWPASSAGEREMAKE_UTILS_H
#define NARROWPASSAGEREMAKE_UTILS_H

#include <optional>
#include <cmath>
#include "Types.h"

namespace utils {
    /**
     * @param component
     * @return İçi boşaltılmış küme döndürür
     */
    Component borderizeComponent(const Component &component)
    {
        // TODO: contour
        return {};
    }

    /**
     * @param p1
     * @param p2
     * @return İki nokta arasındaki mesafeyi döndürür.
     */
    double getDistance(const Point& p1, const Point& p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    /** @brief Birbirine komşuluğu bulunan harita elemanlarını gruplar
     * @param connectedComponents Sonuçlar içine yazılacak değişken
     * @param map Kümelenecek harita
     * @param componentValue Kümelenecek eleman türü (0: free space, 1: engel)
     * @param dir Komşuluk yön sayısı
     */
    void getConnectedComponents(std::vector<Component> &connectedComponents,
                                  const Map& map,
                                  bool componentValue = 1,
                                  int dir = 8)
    {
        int rows = map.size();
        int cols = map.empty() ? 0 : map[0].size();

        // 8 bit unsigned integer tek kanal (grayscale) bir matris oluşturur.
        cv::Mat cvMap(rows, cols, CV_8UC1);

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                cvMap.at<uchar>(i, j) = map[i][j] * 255; // 1 değerlerini 255 yapar

        // Kümelenecek değer 0 ise cvMap'i tersine çevir.
        if (componentValue == 0)
            cv::bitwise_not(cvMap, cvMap);

        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(cvMap, labels, stats, centroids, dir, CV_32S, cv::CCL_DEFAULT);

        // connectedComponents için alan ayır
        connectedComponents.resize(num_labels-1);
        for (int i = 0; i < connectedComponents.size(); i++)
        {
            int area = stats.at<int>(i+1, cv::CC_STAT_AREA);
            connectedComponents[i].reserve(area);
        }

        for(int y = 0; y < labels.rows; y++)
        {
            for(int x = 0; x < labels.cols; x++)
            {
                int label = labels.at<int>(y, x);
                if(label > 0)
                    connectedComponents[label-1].push_back({x, y});
            }
        }
    }
}

#endif //NARROWPASSAGEREMAKE_UTILS_H
