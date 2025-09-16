#include <iostream>
#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#include "model.h"
#include "tgaimage.h"

Eigen::Vector4i white = {255, 255, 255, 255}; // attention, BGRA order
Eigen::Vector4i green = {0, 255, 0, 255};
Eigen::Vector4i red = {0, 0, 255, 255};
Eigen::Vector4i blue = {255, 128, 64, 255};
Eigen::Vector4i yellow = {0, 200, 255, 255};


constexpr int width = 800;
constexpr int height = 800;

#define MY_PI 3.1415926
#define TWO_PI (2.0* MY_PI)
inline double DEG2RAD(double deg) { return deg * MY_PI / 180; }

enum AAType {
    NONE, SSAA, MSAA, FXAA, TAA
};

TGAColor get_TGAColor(Eigen::Vector4i color) {
    TGAColor tgacolor = {static_cast<unsigned char>(color[0]),
        static_cast<unsigned char>(color[1]),
        static_cast<unsigned char>(color[2]),
        static_cast<unsigned char>(color[3])};
    return tgacolor;
}

/**
 * 视图变换
 * @param eye_pos 相机位置
 * @param look_at 视角方向
 * @param t 相机头朝向
 **/
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos, Eigen::Vector3f look_at, Eigen::Vector3f up) {
    Eigen::Vector3f cameraDirection = (eye_pos - look_at).normalized();
    Eigen::Vector3f right = up.cross(cameraDirection).normalized();
    Eigen::Vector3f cameraUp = cameraDirection.cross(right);


    Eigen::Matrix4f view;
    view << right.x(), right.y(), right.z(), 0,
            cameraUp.x(), cameraUp.y(), cameraUp.z(), 0,
            -look_at.x(), -look_at.y(), -look_at.z(), 0,
            0, 0, 0, 1;


    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos.x(),
            0, 1, 0, -eye_pos.y(),
            0, 0, 1, -eye_pos.z(),
            0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 罗德里格斯旋转公式
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {
    // Use Rodrigues rotation formula
    float radian = DEG2RAD(angle);
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f M;
    Eigen::Matrix3f Rk;
    Rk << 0, -axis[2], axis[1],
            axis[2], 0, -axis[0],
            -axis[1], axis[0], 0;

    M = I + (1 - cos(radian)) * Rk * Rk + sin(radian) * Rk;

    model << M(0, 0), M(0, 1), M(0, 2), 0,
            M(1, 0), M(1, 1), M(1, 2), 0,
            M(2, 0), M(2, 1), M(2, 2), 0,
            0, 0, 0, 1;
    return model;
}

/**
 *  模型变换
 *  @param s 缩放矩阵
 *  @param t 平移矩阵
 *  @param angle 旋转角度
 *  @param rot_vec 旋转轴
 */
Eigen::Matrix4f get_model_matrix(Eigen::Vector3f s, Eigen::Vector3f t, Eigen::Vector3f rot_vec, float angle) {
    Eigen::Matrix4f rotation;
    rotation = get_rotation(rot_vec, angle);

    Eigen::Matrix4f scale;
    scale << s.x(), 0, 0, 0,
            0, s.y(), 0, 0,
            0, 0, s.z(), 0,
            0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, t.x(),
            0, 1, 0, t.y(),
            0, 0, 1, t.z(),
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
    Eigen::Matrix4f projection;
    float top = -tan(DEG2RAD(eye_fov / 2.0f) * abs(zNear));
    float right = top * aspect_ratio;

    projection << zNear / right, 0, 0, 0,
            0, zNear / top, 0, 0,
            0, 0, (zNear + zFar) / (zNear - zFar), (2 * zNear * zFar) / (zFar - zNear),
            0, 0, 1, 0;
    return projection;
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

std::tuple<double, double, double> computer_lambda(float x, float y, float ax, float ay, float bx, float by, float cx,
                                                   float cy) {
    double total_area = signed_triangle_area(ax, ay, bx, by, cx, cy);

    double lambda_1 = signed_triangle_area(x, y, bx, by, cx, cy) / total_area;
    double lambda_2 = signed_triangle_area(x, y, cx, cy, ax, ay) / total_area;
    double lambda_3 = signed_triangle_area(x, y, ax, ay, bx, by) / total_area;
    return {lambda_1, lambda_2, lambda_3};
}

bool in_triangle(double lambda_1, double lambda_2, double lambda_3) {
    if (lambda_1 < 0 || lambda_2 < 0 || lambda_3 < 0) return false;
    return true;
}


void draw_triangle(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz,
                   TGAImage &framebuffer, TGAImage &zbuffer, Eigen::Vector4i color,
                   AAType type) {
    float x_max, x_min, y_max, y_min;
    x_max = std::max(ax, bx);
    x_max = std::max(x_max, cx);
    x_min = std::min(ax, bx);
    x_min = std::min(x_min, cx);
    y_max = std::max(ay, by);
    y_max = std::max(y_max, cy);
    y_min = std::min(ay, by);
    y_min = std::min(y_min, cy);
    // unsigned char z = static_cast<unsigned char>(az);
    for (int y = y_min; y <= y_max; y++) {
        for (int x = x_min; x <= x_max; x++) {
            auto [lambda_1, lambda_2, lambda_3] = computer_lambda(x, y, ax, ay, bx, by, cx, cy);

            if (in_triangle(lambda_1, lambda_2, lambda_3)) {
                unsigned char z = static_cast<unsigned char>(lambda_1 * az + lambda_2 * bz + lambda_3 * cz);
                if (z <= zbuffer.get(x, y)[0]) continue;
                zbuffer.set(x, y, {z});
                framebuffer.set(x, y, get_TGAColor(color));

            }


        }
    }
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}

Eigen::Vector4f MVP_tr(Eigen::Vector3f v, float angle, Eigen::Vector3f eye_pos, float eye_fov, float aspect_ratio,
                       float zNear, float zFar) {
    Eigen::Vector3f s = {100, 100, 100};
    Eigen::Vector3f t = {-100, -100, 0};
    Eigen::Vector3f rot_vec = {0, 0, 1};

    Eigen::Vector3f look_at = {0, 0, 0};
    Eigen::Vector3f look_at_t = {0, 1, 0};

    Eigen::Matrix4f model_matrix = get_model_matrix(s, t, rot_vec, angle);
    Eigen::Matrix4f view_matrix = get_view_matrix(eye_pos, look_at, look_at_t);
    Eigen::Matrix4f projection_matrix = get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar);

    Eigen::Matrix4f mvp = projection_matrix * view_matrix * model_matrix;

    return mvp * to_vec4(v, 1.0f);
};


int main(int argc, char **argv) {
    Model model("/Users/huangkaicheng/Desktop/github/render/models/spot/spot_triangulated_good.obj");
    std::string filename = "./framebuffer.png";


    // 定义摄像机z轴距离
    Eigen::Vector3f eye_pos = {0, 0, 3.0};
    // 定义初始角度
    float angle = 180.0;
    // 视角度数
    float eye_fov = 45.0;
    // 长宽比
    float aspect_ratio = 1;
    // 摄像机距离
    float zNear = 0.1;
    // 最远距离
    float zFar = 100.0;


    TGAImage framebuffer(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

    for (int face = 0; face < model.nfaces(); face++) {

        Eigen::Vector4f a = MVP_tr(model.vert(face, 0), angle, eye_pos, eye_fov, aspect_ratio, zNear, zFar);
        Eigen::Vector4f b = MVP_tr(model.vert(face, 1), angle, eye_pos, eye_fov, aspect_ratio, zNear, zFar);
        Eigen::Vector4f c = MVP_tr(model.vert(face, 2), angle, eye_pos, eye_fov, aspect_ratio, zNear, zFar);


        // draw_line(a.x(), a.y(), b.x(), b.y(), framebuffer, get_TGAColor(yellow));
        // draw_line(a.x(), a.y(), c.x(), c.y(), framebuffer, get_TGAColor(yellow));
        // draw_line(b.x(), b.y(), c.x(), c.y(), framebuffer, get_TGAColor(yellow));

        draw_triangle(a.x(), a.y(), a.z(), b.x(), b.y(), b.z(), c.x(), c.y(), c.z(), framebuffer, zbuffer, yellow,
        NONE);
    }


    framebuffer.write_tga_file("framebuffer.tga");
    zbuffer.write_tga_file("zbuffer.tga");

    return 0;
}
