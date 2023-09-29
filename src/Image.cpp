#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"
#include "Image.h"
#include <iostream>

Image::Image(const std::string& filename) {
    int width, height, channels;
    unsigned char* rawData = stbi_load(filename.c_str(), &width, &height, &channels, 1);  // grayscale olarak yükleniyor

    if (rawData) {
        for (int i = 0; i < height; ++i) {
            std::vector<bool> row;
            for (int j = 0; j < width; ++j) {
                int pixelValue;
                if (rawData[i*width + j] < 230)
                    pixelValue = 1;
                else
                    pixelValue = 0;
                // int pixelValue = rawData[i * width + j] ? 0 : 1;
                row.push_back(pixelValue);
            }
            data.push_back(row);
        }
        stbi_image_free(rawData);
    } else {
        std::cerr << filename << " yüklenirken hata oluştu!" << std::endl;
    }
}

void Image::writeBoolMatrixToPNG(const std::vector<std::vector<bool>>& data, const std::string& filename) {
    int width = data[0].size();
    int height = data.size();

    // RGBA formatında raw veri oluştur
    std::vector<unsigned char> raw_data(height * width * 4);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = (y * width + x) * 4;

            // Siyah-beyaz renk kodlaması
            unsigned char color = data[y][x] ? 255 : 0;          

            raw_data[index] = color;     // R
            raw_data[index + 1] = color; // G
            raw_data[index + 2] = color; // B
            raw_data[index + 3] = 255;   // A (Tam saydamlık)
        }
    }

    if (!stbi_write_png(filename.c_str(), width, height, 4, raw_data.data(), width * 4)) {
        std::cerr << "Error writing image using stb_image_write." << std::endl;
    }
}

void Image::writeBoolMatrixToPNG(const std::vector<std::vector<int>>& data, const std::string& filename) {
    int width = data[0].size();
    int height = data.size();

    // RGBA formatında raw veri oluştur
    std::vector<unsigned char> raw_data(height * width * 4);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = (y * width + x) * 4;

            // Siyah-beyaz renk kodlaması
            unsigned char color = data[y][x] == 0 ? 255 : 0;          

            raw_data[index] = color;     // R
            raw_data[index + 1] = color; // G
            raw_data[index + 2] = color; // B
            raw_data[index + 3] = 255;   // A (Tam saydamlık)
        }
    }

    if (!stbi_write_png(filename.c_str(), width, height, 4, raw_data.data(), width * 4)) {
        std::cerr << "Error writing image using stb_image_write." << std::endl;
    }
}

int Image::getWidth() const { 
    return width; 
}

int Image::getHeight() const { 
    return height; 
}

int Image::getPixel(int x, int y) const { 
    return data[y][x]; 
}

