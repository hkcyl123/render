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


Eigen::Matrix4f ModelView, Viewport, Perspective;

// 物体旋转角度
Eigen::Vector3f model_tran(Eigen::Vector3f v, double angel) {
    double a = M_PI * angel / 180.0;
    Eigen::Matrix3f Ry;
    Ry << std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a);
    return Ry * v;
}

// 摄像机方向
void lookat(const Eigen::Vector3f eye, const Eigen::Vector3f center, const Eigen::Vector3f up) {
    Eigen::Vector3f n = (eye - center).normalized();
    Eigen::Vector3f l = up.cross(n).normalized();
    Eigen::Vector3f m = n.cross(l).normalized();
    Eigen::Matrix4f model, view;
    model << l.x(), l.y(), l.z(), 0, m.x(), m.y(), m.z(), 0, n.x(), n.y(), n.z(), 0, 0, 0, 0, 1;
    view << 1, 0, 0, -center.x(), 0, 1, 0, -center.y(), 0, 0, 1, -center.z(), 0, 0, 0, 1;
    ModelView = model * view;
}

void viewport(const int x, const int y, const int w, const int h) {
    Viewport << w / 2., 0, 0, x + w / 2., 0, h / 2., 0, y + h / 2., 0, 0, 1, 0, 0, 0, 0, 1;
}

// 透视变换
void perspective(const double f) {
    Perspective << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -1 / f, 1;
}

Eigen::Vector3f perspective(Eigen::Vector3f v, double c) {
    return v / (1 - v.z() / c);
}


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


void draw_triangle(Eigen::Vector4f a, Eigen::Vector4f b, Eigen::Vector4f c,
                   TGAImage &framebuffer, TGAImage &zbuffer, TGAColor color, AAType type) {
    Eigen::Vector4f ndc[3] = {a / a.w(), b / b.w(), c / c.w()}; // normalized device coordinates
    Eigen::Vector4f product_1 = Viewport * ndc[0];
    Eigen::Vector4f product_2 = Viewport * ndc[1];
    Eigen::Vector4f product_3 = Viewport * ndc[2];
    // Eigen::Vector2f screen[3] = { ().xy(), (Viewport*ndc[1]).xy(), (Viewport*ndc[2]).xy() }; // screen coordinates
    Eigen::Vector2f screen[3] = {
        {product_1.x(), product_1.y()}, {product_2.x(), product_2.y()}, {product_3.x(), product_3.y()}
    };

    Eigen::Matrix3f ABC;
    ABC << screen[0].x(), screen[0].y(), 1., screen[1].x(), screen[1].y(), 1., screen[2].x(), screen[2].y(), 1.;
    // if (ABC.det() < 1) return; // backface culling + discarding triangles that cover less than a pixel


    auto [bbminx,bbmaxx] = std::minmax({screen[0].x(), screen[1].x(), screen[2].x()}); // bounding box for the triangle
    auto [bbminy,bbmaxy] = std::minmax({screen[0].y(), screen[1].y(), screen[2].y()});
    // defined by its top left and bottom right corners

    // unsigned char z = static_cast<unsigned char>(az);

    for (int x = std::max<int>(bbminx, 0); x <= std::min<int>(bbmaxx, framebuffer.width() - 1); x++) {
        // clip the bounding box by the screen
        for (int y = std::max<int>(bbminy, 0); y <= std::min<int>(bbmaxy, framebuffer.height() - 1); y++) {
            Eigen::Vector3f bc = ABC.inverse().transpose() * Eigen::Vector3f{
                                     static_cast<float>(x), static_cast<float>(y), 1.
                                 }; // barycentric coordinates of {x,y} w.r.t the triangle
            if (bc.x() < 0 || bc.y() < 0 || bc.z() < 0) continue;
            Eigen::Vector3f zb = {ndc[0].z(), ndc[1].z(), ndc[2].z()};
            // negative barycentric coordinate => the pixel is outside the triangle
            float z = bc.dot(zb);
            if (z <= zbuffer.get(x, y)[0]) continue;
            // zbuffer.set(x, y, {z});

            framebuffer.set(x, y, color);
        }
    }

    // for (int y = y_min; y <= y_max; y++) {
    //     for (int x = x_min; x <= x_max; x++) {
    //         if (type == SSAA) {
    //             int left_top, right_top, left_bottom, right_bottom;
    //
    //             left_top = in_triangle(x, y, ax, ay, bx, by, cx, cy);
    //             right_top = in_triangle(x, y, ax, ay, bx, by, cx, cy);
    //             left_bottom = in_triangle(x, y, ax, ay, bx, by, cx, cy);
    //             right_bottom = in_triangle(x, y, ax, ay, bx, by, cx, cy);
    //         }
    //
    //         if (in_triangle(x, y, ax, ay, bx, by, cx, cy)) {
    //             if (z <= zbuffer.get(x, y)[0]) continue;
    //             // unsigned char z = static_cast<unsigned char>(1 * az + 0 * bz + 0 * cz);
    //             zbuffer.set(x, y, {z});
    //             framebuffer.set(x, y, color);
    //         }
    //     }
    // }
}

Eigen::Vector4f to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}


int main(int argc, char **argv) {
    Model model("/Users/huangkaicheng/Desktop/github/render/models/spot/spot_triangulated_good.obj");


    TGAImage framebuffer(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);


    Eigen::Vector3f eye{-1, 0, 2}; // camera position
    Eigen::Vector3f center{0, 0, 0}; // camera direction
    Eigen::Vector3f up{0, 1, 0}; // camera up vector

    lookat(eye, center, up); // build the ModelView   matrix
    perspective((eye - center).norm()); // build the Perspective matrix
    viewport(width / 16, height / 16, width * 7 / 8, height * 7 / 8); // build the Viewport    matrix


    double angel = 90.0;
    double zNear = 0.1;
    double zFar = 100.0;

    for (int face = 0; face < model.nfaces(); face++) {
        Eigen::Vector4f a = Perspective * ModelView * to_vec4(model.vert(face, 0));
        Eigen::Vector4f b = Perspective * ModelView * to_vec4(model.vert(face, 0));
        Eigen::Vector4f c = Perspective * ModelView * to_vec4(model.vert(face, 0));

        // auto [ax, ay, az] = project(perspective(model_tran(model.vert(face, 0), angel), zFar - zNear));
        // auto [bx, by, bz] = project(perspective(model_tran(model.vert(face, 1), angel), zFar - zNear));
        // auto [cx, cy, cz] = project(perspective(model_tran(model.vert(face, 2), angel), zFar - zNear));


        draw_triangle(a, b, c, framebuffer, zbuffer, yellow, NONE);
    }


    framebuffer.write_tga_file("framebuffer.tga");
    zbuffer.write_tga_file("zbuffer.tga");

    return 0;
}
