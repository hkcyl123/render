#include <iostream>
#include <Eigen/Eigen>

#include "model.h"
#include "tgaimage.h"

constexpr TGAColor white = {255, 255, 255, 255}; // attention, BGRA order
constexpr TGAColor green = {0, 255, 0, 255};
constexpr TGAColor red = {0, 0, 255, 255};
constexpr TGAColor blue = {255, 128, 64, 255};
constexpr TGAColor yellow = {0, 200, 255, 255};


constexpr int width = 800;
constexpr int height = 800;

enum AAType {
    NONE, SSAA, MSAA, FXAA, TAA
};

void draw_line(float x_1, float y_1, float x_2, float y_2, TGAImage &framebuffer, TGAColor color) {
    bool steep = std::abs(x_1 - x_2) < std::abs(y_1 - y_2);
    if (steep) {
        // if the line is steep, we transpose the image
        std::swap(x_1, y_1);
        std::swap(x_2, y_2);
    }
    if (x_1 > x_2) {
        // make it left−to−right
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

double signed_triangle_area(float ax, float ay, float bx, float by, float cx, float cy) {
    return 0.5 * ((by - ay) * (bx + ax) + (cy - by) * (cx + bx) + (ay - cy) * (ax + cx));
}

bool in_triangle(float x, float y, float ax, float ay, float bx, float by, float cx, float cy) {
    double total_area = signed_triangle_area(ax, ay, bx, by, cx, cy);

    double lambda_1 = signed_triangle_area(x, y, bx, by, cx, cy) / total_area;
    double lambda_2 = signed_triangle_area(x, y, cx, cy, ax, ay) / total_area;
    double lambda_3 = signed_triangle_area(x, y, ax, ay, bx, by) / total_area;

    if (lambda_1 < 0 || lambda_2 < 0 || lambda_3 < 0) return false;
    // negative barycentric coordinate => the pixel is outside the triangle

    return true;
}


void draw_triangle(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz,
                   TGAImage &framebuffer, TGAImage &zbuffer, TGAColor color, AAType type) {
    float x_max, x_min, y_max, y_min;
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
            if (type == SSAA) {
                int left_top, right_top, left_bottom, right_bottom;

                left_top = in_triangle(x, y, ax, ay, bx, by, cx, cy);
                right_top = in_triangle(x, y, ax, ay, bx, by, cx, cy);
                left_bottom = in_triangle(x, y, ax, ay, bx, by, cx, cy);
                right_bottom = in_triangle(x, y, ax, ay, bx, by, cx, cy);
            }

            if (in_triangle(x, y, ax, ay, bx, by, cx, cy)) {
                if (z <= zbuffer.get(x, y)[0]) continue;
                // unsigned char z = static_cast<unsigned char>(1 * az + 0 * bz + 0 * cz);
                zbuffer.set(x, y, {z});
                framebuffer.set(x, y, color);
            }
        }
    }
}

std::tuple<int,int> project(Eigen::Vector3f v) { // First of all, (x,y) is an orthogonal projection of the vector (x,y,z).
    return { (v.x() + 1.) *  width/2,   // Second, since the input models are scaled to have fit in the [-1,1]^3 world coordinates,
             (v.y() + 1.) * height/2 }; // we want to shift the vector (x,y) and then scale it to span the entire screen.
}


int main(int argc, char **argv) {
    Model model("/Users/huangkaicheng/Desktop/github/render/models/spot/spot_triangulated_good.obj");

    TGAImage framebuffer(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

    for (int face = 0; face < model.nfaces(); face++) {

        auto [ax, ay] = project(model.vert(face, 0));
        auto [bx, by] = project(model.vert(face, 1));
        auto [cx, cy] = project(model.vert(face, 2));

        draw_line(ax, ay, bx, by, framebuffer, yellow);
        draw_line(cx, cy, bx, by, framebuffer, yellow);
        draw_line(ax, ay, cx, cy, framebuffer, yellow);

        // draw_triangle(ax, ay, az, bx, by, bz, cx, cy, cz, framebuffer, zbuffer, yellow, NONE);
    }


    framebuffer.write_tga_file("framebuffer.tga");

    return 0;
}
