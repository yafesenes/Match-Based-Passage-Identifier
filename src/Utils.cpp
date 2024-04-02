#include "Utils.h"

double utils::getDistance(const Point& p1, const Point& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

void utils::getConnectedComponents(std::vector<Component> &connectedComponents,
                              const Map& map,
                              bool componentValue,
                              int dir,
                              Point refPoint)
{
    int rows = map.size();
    int cols = map.empty() ? 0 : map[0].size();

    cv::Mat cvMap(rows, cols, CV_8UC1);

    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            cvMap.at<uchar>(i, j) = map[i][j] * 255;

    if (componentValue == 0)
        cv::bitwise_not(cvMap, cvMap);

    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(cvMap, labels, stats, centroids, dir, CV_32S, cv::CCL_DEFAULT);

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
                connectedComponents[label-1].push_back({x + refPoint.x, y+refPoint.y});
        }
    }
}

std::vector<Point> utils::bresenham(Point p0, Point p1) {
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

    return line;
}

Rect  utils::ComponentToRect(const std::vector<Point> &Component)
{
    Point TopLeft = Component[0];
    Point BotRight = Component[0];

    for (const Point &p:Component)
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

    return std::make_pair(TopLeft, BotRight);
}

Convex utils::ConvexHull(Component P) {
    int n = P.size(), k = 0;
    Component H(2 * n);

    std::sort(P.begin(), P.end(), compare);

    for (int i = 0; i < n; ++i) {
        while (k >= 2 && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
        H[k++] = P[i];
    }

    for (int i = n - 2, t = k + 1; i >= 0; i--) {
        while (k >= t && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
        H[k++] = P[i];
    }

    H.resize(k - 1);

    return H;
}

bool utils::compare(Point p1, Point p2) {
    return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
}

int utils::cross(const Point &O, const Point &A, const Point &B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

Component utils::borderizeComponent(const Component& component, const Map& occupancyMap, bool componentValue)
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
                int x = j.x + directionsX[k];
                int y = j.y + directionsY[k];

                if (y >= occupancyMap.size() || x >= occupancyMap[y].size() || x < 0 || y < 0)
                    continue;

                if (occupancyMap[y][x] == !componentValue)
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

bool utils::isInsideConvex(const Convex &convexHull, const Point &point) {
    size_t n = convexHull.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;

        if (cross(convexHull[i], convexHull[j], point) < 0) {
            return false;
        }
    }
    return true;
}

void utils::ClusterConvexPolygon(const std::vector<Point> &polygon, const std::vector<Point> &points, std::pair<std::vector<Point>, std::vector<Point>>& PolygonPoints)
{
    size_t n = polygon.size();

    for(const Point &point : points) {
        int sign = 0;
        bool inside = true;

        for(size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            int cp = cross(polygon[i], polygon[j], point);

            if(cp == 0) continue;

            int newSign = (cp > 0) ? 1 : -1;

            if(sign == 0) {
                sign = newSign;
            } else if(sign != newSign) {
                inside = false;
                break;
            }
        }

        if(inside) {
            PolygonPoints.second.push_back(point);
        } else {
            PolygonPoints.first.push_back(point);
        }
    }
}
