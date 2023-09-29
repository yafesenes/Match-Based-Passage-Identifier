#include "Utils.h"

std::vector<Component> utils::borderizeComponents(const Map &map)
{
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
                int x = j.x + directionsX[k] - refValue.x;
                int y = j.y + directionsY[k] - refValue.y;

                if (y >= map.size() || x >= map[y].size() || x < 0 || y < 0)
                    continue;

                if (map[y][x] == !ComponentValue)
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
    return newComponent;
}

double utils::getDistance(const Point& p1, const Point& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

void utils::getConnectedComponents(std::vector<Component> &connectedComponents,
                              const Map& map,
                              bool componentValue,
                              int dir)
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
