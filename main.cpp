#include <cmath>
#include "tgaimage.h"

constexpr TGAColor white = {255, 255, 255, 255}; // attention, BGRA order
constexpr TGAColor green = {0, 255, 0, 255};
constexpr TGAColor red = {0, 0, 255, 255};
constexpr TGAColor blue = {255, 128, 64, 255};
constexpr TGAColor yellow = {0, 200, 255, 255};

void draw_line(int x_1, int y_1, int x_2, int y_2, TGAImage &framebuffer, TGAColor color) {
    bool steep = std::abs(x_1-x_2) < std::abs(y_1-y_2);
    if (steep) { // if the line is steep, we transpose the image
        std::swap(x_1, y_1);
        std::swap(x_2, y_2);
    }
    if (x_1>x_2) { // make it left−to−right
        std::swap(x_1, x_2);
        std::swap(y_1, y_2);
    }
    for (int x = x_1; x <= x_2; x++) {
        float t = (x - x_1) / static_cast<float>(x_2 - x_1);
        int y = std::round(y_1 + (y_2 - y_1) * t);
        if (steep) // if transposed, de−transpose
            framebuffer.set(y, x, color);
        else
            framebuffer.set(x, y, color);
    }
}

int main(int argc, char **argv) {
    constexpr int width = 64;
    constexpr int height = 64;
    TGAImage framebuffer(width, height, TGAImage::RGB);

    int ax = 7, ay = 3;
    int bx = 12, by = 37;
    int cx = 62, cy = 53;

    draw_line(ax, ay, bx, by, framebuffer, green);
    draw_line(bx, by, cx, cy, framebuffer, blue);
    draw_line(cx, cy, ax, ay, framebuffer, yellow);

    framebuffer.set(ax, ay, white);
    framebuffer.set(bx, by, white);
    framebuffer.set(cx, cy, white);

    framebuffer.write_tga_file("framebuffer.tga");
    return 0;
}
