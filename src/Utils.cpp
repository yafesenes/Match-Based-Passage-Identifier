#include "Utils.h"

double utils::getDistance(const Point& p1, const Point& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

void utils::getConnectedComponents(std::vector<Component> &connectedComponents,
                              const Map& map,
                              bool componentValue,
                              int dir)
{
    tic("utils::getConnectedComponents");
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
    toc("utils::getConnectedComponents");
}

std::vector<Point> utils::bresenham(Point p0, Point p1) {
    tic("utils::bresenham");
    std::vector<Point> line;

    int dx = std::abs(p1.x - p0.x);
    int dy = std::abs(p1.y - p0.y);

    int sx = (p0.x < p1.x) ? 1 : -1;
    int sy = (p0.y < p1.y) ? 1 : -1;

    int err = dx - dy;

    while (true) {
        if (p0.x == p1.x && p0.y == p1.y)
            break;

        int e2 = 2 * err;

        if (e2 > -dy) {
            err -= dy;
            p0.x += sx;
        }

        if (e2 < dx) {
            err += dx;
            p0.y += sy;
        }

        if (!(p0.x == p1.x && p0.y == p1.y))
            line.push_back(p0);
    }

    toc("utils::bresenham");
    return line;
}

//TODO: It must be filled!
std::vector<Component> utils::tearComponent(const Component& c)
{
    return {};
}