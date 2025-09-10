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

double signed_triangle_area(int ax, int ay, int bx, int by, int cx, int cy) {
    return 0.5*((by-ay)*(bx+ax) + (cy-by)*(cx+bx) + (ay-cy)*(ax+cx));
}

bool in_triangle(int x, int y, int ax, int ay, int bx, int by, int cx, int cy) {

    double total_area = signed_triangle_area(ax, ay, bx, by, cx, cy);

    double lambda_1 = signed_triangle_area(x, y, bx, by, cx, cy) / total_area;
    double lambda_2 = signed_triangle_area(x, y, cx, cy, ax, ay) / total_area;
    double lambda_3 = signed_triangle_area(x, y, ax, ay, bx, by) / total_area;
    if (lambda_1<0 || lambda_2<0 || lambda_3<0) return false; // negative barycentric coordinate => the pixel is outside the triangle

    return true;
}

void draw_triangle(int ax, int ay, int az, int bx, int by, int bz, int cx, int cy, int cz, TGAImage &framebuffer,TGAImage &zbuffer, TGAColor color) {
    int x_max, x_min, y_max, y_min;
    x_max = std::max(ax, bx);
    x_max = std::max(x_max, cx);
    x_min = std::min(ax, bx);
    x_min = std::min(x_min, cx);
    y_max = std::max(ay, by);
    y_max = std::max(y_max, cy);
    y_min = std::min(ay, by);
    y_min = std::min(y_min, cy);
    unsigned char z = static_cast<unsigned char>(az);
    for (int y = y_min; y <= y_max; y++) {
        for (int x = x_min; x <= x_max; x++) {
            if (in_triangle(x, y, ax, ay, bx, by, cx, cy)) {

                if (z <= zbuffer.get(x, y)[0]) continue;
                // unsigned char z = static_cast<unsigned char>(1 * az + 0 * bz + 0 * cz);
                zbuffer.set(x, y, {z});
                framebuffer.set(x, y, color);
            }
        }
    }
}

int main(int argc, char **argv) {
    constexpr int width = 128;
    constexpr int height = 128;

    TGAImage framebuffer(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

    int ax = 7, ay = 3, az = 50;
    int bx = 12, by = 37, bz = 50;
    int cx = 62, cy = 53, cz = 50;

    int ax_1= 120, ay_1 = 35, az_1 = 30;
    int bx_1 = 90, by_1 = 5, bz_1 = 30;
    int cx_1 = 45, cy_1 = 110, cz_1 = 30;

    int ax_2 = 115, ay_2 = 3, az_2 = 20;
    int bx_2 = 80, by_2 = 37, bz_2 = 20;
    int cx_2 = 85, cy_2 = 53, cz_2 = 20;

    draw_triangle(ax, ay, az, bx, by, bz, cx, cy, cz, framebuffer, zbuffer, yellow);
    draw_triangle(ax_1, ay_1, az_1, bx_1, by_1, bz_1, cx_1, cy_1, cz_1, framebuffer, zbuffer, green);
    draw_triangle(ax_2, ay_2, az_2, bx_2, by_2, bz_2, cx_2, cy_2, cz_2, framebuffer, zbuffer, red);

    framebuffer.write_tga_file("framebuffer.tga");
    return 0;
}
