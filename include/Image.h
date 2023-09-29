#ifndef IMAGE_CLASS_H
#define IMAGE_CLASS_H

#include <vector>
#include <string>

class Image {
private:
    std::vector<std::vector<bool>> data;
    int width;
    int height;

public:
    Image(const std::string& filename);

    int getWidth() const;
    int getHeight() const;
    int getPixel(int x, int y) const;
    std::vector<std::vector<bool>> getData() const { return data; }

    static void writeBoolMatrixToPNG(const std::vector<std::vector<bool>>& data, const std::string& filename);
    static void writeBoolMatrixToPNG(const std::vector<std::vector<int>>& data, const std::string& filename);
};

#endif
