//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include "Circle.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

rst::pos_buf_id rst::rasterizer::load_positions(
        const std::vector<Eigen::Vector3f> &positions) {
    auto id = get_next_id();        // 获取下一个三角形渲染
    pos_buf.emplace(id, positions); // 将要渲染的三角形的位置存入pos_buf

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(
        const std::vector<Eigen::Vector3i> &indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end) {
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;
    dy = y2 - y1;
    dx1 = fabs(dx);
    dy1 = fabs(dy);
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if (dy1 <= dx1) {
        if (dx >= 0) {
            x = x1;
            y = y1;
            xe = x2;
        } else {
            x = x2;
            y = y2;
            xe = x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; x < xe; i++) {
            x = x + 1;
            if (px < 0) {
                px = px + 2 * dy1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                    y = y + 1;
                } else {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, line_color);
        }
    } else {
        if (dy >= 0) {
            x = x1;
            y = y1;
            ye = y2;
        } else {
            x = x2;
            y = y2;
            ye = y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; y < ye; i++) {
            y = y + 1;
            if (py <= 0) {
                py = py + 2 * dx1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                    x = x + 1;
                } else {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, line_color);
        }
    }
}

// 先假设圆在原点处，然后平移到圆心处
void rst::rasterizer::draw_circle_line(Eigen::Vector3f point, int r) {
    Eigen::Vector3f line_color = {255, 255, 0};
    int p, y, x;
    int y0 = r;
    int p0 = 5 / 4 - r;
    y = y0;
    p = p0;
    Eigen::Vector3f point0 = Eigen::Vector3f(0, y0, 1.0f);
    set_pixel(point0, line_color);
    for (x = 0; x < y; ++x) {
        if (p > 0) { // 如果P > 0 说明yk+1 = yk - 1,pk+1 = pk + 2xk + 1 - 2yk
            p = p + 2 * x + 1 - 2 * y;
            y = y - 1;
        } else { // yk+1 = yk pk+1 = pk + 2xk + 3
            p = p + 2 * x + 3;
            y = y;
        }

        Eigen::Vector3f point1 = Eigen::Vector3f(x + point.x(), y + point.y(), 1.0f);
        // 然后画xy对称的坐标
        Eigen::Vector3f point2 = Eigen::Vector3f(y + point.x(), x + point.y(), 1.0f);;
        // 关于x 对称的坐标
        Eigen::Vector3f point3 = Eigen::Vector3f(x + point.x(), -y + point.y(), 1.0f);
        Eigen::Vector3f point4 = Eigen::Vector3f(y + point.x(), -x + point.y(), 1.0f);
        // 关于y对称的坐标
        Eigen::Vector3f point5 = Eigen::Vector3f(-x + point.x(), y + point.y(), 1.0f);
        Eigen::Vector3f point6 = Eigen::Vector3f(-y + point.x(), x + point.y(), 1.0f);;
        Eigen::Vector3f point7 = Eigen::Vector3f(-x + point.x(), -y + point.y(), 1.0f);
        Eigen::Vector3f point8 = Eigen::Vector3f(-y + point.x(), -x + point.y(), 1.0f);
        set_pixel(point1, line_color);
        set_pixel(point2, line_color);
        set_pixel(point3, line_color);
        set_pixel(point4, line_color);
        set_pixel(point5, line_color);
        set_pixel(point6, line_color);
        set_pixel(point7, line_color);
        set_pixel(point8, line_color);
    }
}

/**
 *
 * @param v 中心坐标，先原点，后面再移过去
 * @param d1 长轴的一半
 * @param d2 短轴的一半
 */
void rst::rasterizer::draw_elliptic_line(Vector3f v, int d1, int d2) {
    Eigen::Vector3f line_color = {255, 255, 0};
    Eigen::Vector3f point = Eigen::Vector3f( v.x(), v.y(), 1.0f);
    set_pixel(point, line_color);
    int rx_squ = d1 * d1;
    int ry_squ = d2 * d2;
    int x1, y1;
    double p1k;
    y1 = d2;
    x1 = 0;
    p1k = ry_squ - rx_squ * d2 + 0.25 * rx_squ;
    for (; ry_squ * x1 < rx_squ * y1; x1++) {
        if (p1k < 0) {
            y1 = y1;
            p1k = p1k + ry_squ * (2 * x1 + 3);
        } else {
            p1k = p1k + ry_squ * (2 * x1 + 3) - rx_squ * (2 * y1 - 2);
            y1 = y1 - 1;
        }
        // 设置点和它的对称点
        Eigen::Vector3f point1 = Eigen::Vector3f(x1 + v.x(), y1 + v.y(), 1.0f);
        // 关于x轴对称
        Eigen::Vector3f point2 = Eigen::Vector3f(x1 + v.x(), -y1 + v.y(), 1.0f);
        // 关于Y轴对称
        Eigen::Vector3f point3 = Eigen::Vector3f(-x1 + v.x(), y1 + v.y(), 1.0f);
        Eigen::Vector3f point4 = Eigen::Vector3f(-x1 + v.x(), -y1 + v.y(), 1.0f);
        set_pixel(point1, line_color);
        set_pixel(point2, line_color);
        set_pixel(point3, line_color);
        set_pixel(point4, line_color);

    }
    // region2
    int x2, y2;
    double p2k;
    p2k = ry_squ * (x1 + 0.5) * (x1 + 0.5) + rx_squ * (y1 - 1) * (y1 - 1) - rx_squ * ry_squ;
    x2 = x1 - 1;
    y2 = y1;
    for (; y2 >= 0; --y2) {
        if (p2k < 0) {
            p2k = p2k - 2 * rx_squ * (y2 - 1) + rx_squ + ry_squ * (2 * x2 + 2);
            x2 = x2 + 1;
        } else {
            p2k = p2k - 2 * rx_squ * (y2 - 1) + rx_squ;
            x2 = x2 ;
        }
        // 设置点和它的对称点
        Eigen::Vector3f point1 = Eigen::Vector3f(x2 + v.x(), y2 + v.y(), 1.0f);
        // 关于x轴对称
        Eigen::Vector3f point2 = Eigen::Vector3f(x2 + v.x(), -y2 + v.y(), 1.0f);
        // 关于Y轴对称
        Eigen::Vector3f point3 = Eigen::Vector3f(-x2 + v.x(), y2 + v.y(), 1.0f);
        Eigen::Vector3f point4 = Eigen::Vector3f(-x2 + v.x(), -y2 + v.y(), 1.0f);
        set_pixel(point1, line_color);
        set_pixel(point2, line_color);
        set_pixel(point3, line_color);
        set_pixel(point4, line_color);
    }
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type) {
    if (type == rst::Primitive::Triangle) {
        auto &buf = pos_buf[pos_buffer.pos_id]; // 取出存的三角形相应的数据
        auto &ind = ind_buf[ind_buffer.ind_id];

        float f1 = (100 - 0.1) / 2.0; // ?
        float f2 = (100 + 0.1) / 2.0;

        Eigen::Matrix4f mvp = projection * view * model;
        for (auto &i : ind) // 当前三角形
        {
            Triangle t;
            Eigen::Vector4f v[] = {
                    mvp * to_vec4(buf[i[0]], 1.0f), // 将三个点全部化为齐次坐标
                    mvp * to_vec4(buf[i[1]], 1.0f),
                    mvp * to_vec4(buf[i[2]], 1.0f)};

            for (auto &vec : v) // 此处的v是存放了变换后的三个点的坐标,
            {
                vec = vec / vec[3]; // vec[3] 是为了将vec[3]重新化为1.0f
            }

            for (auto &vert : v) // 将三角形放大,方便看
            {
                vert.x() = 0.5 * width * (vert.x() + 1.0);
                vert.y() = 0.5 * height * (vert.y() + 1.0);
                vert.z() = vert.z() * f1 + f2;
            }

            for (int i = 0; i < 3; ++i) {
                t.setVertex(i, v[i].head<3>()); // 更新三角形的新坐标
                // t.setVertex(i, v[i].head<3>());
                // t.setVertex(i, v[i].head<3>());
            }

            t.setColor(0, 255.0, 0.0, 0.0);
            t.setColor(1, 0.0, 255.0, 0.0);
            t.setColor(2, 0.0, 0.0, 255.0);

            rasterize_wireframe(t); // 渲染新三角形
        }
    }
}


void rst::rasterizer::draw_circle(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type, int r) {
    if (type == rst::Primitive::Circle) {
        auto &buf = pos_buf[pos_buffer.pos_id]; // 取出存的圆形相应的数据
        auto &ind = ind_buf[ind_buffer.ind_id];
        Eigen::Matrix4f mvp = projection * view * model;

        for (auto &i : ind) {
            Circle c;
            Eigen::Vector4f v[] = {
                    mvp * to_vec4(buf[i[0]], 1.0f)}; // 先画一个圆
            for (auto &vec : v) {
                vec = vec / vec[3]; // 将齐次坐标的最后一位化为1
            }
            for (auto &vert : v) // 将图形放中间放大
            {
                vert.x() = 0.5 * width + (vert.x() + 1.0);
                vert.y() = 0.5 * height + (vert.y() + 1.0);
            }
            c.setVertex(v[0].head<3>()); // 更新变换后的坐标
            // 然后画1/8弧线
            Eigen::Vector3f line_color = {255, 255, 0};
            set_pixel(c.a(), line_color);
            draw_circle_line(c.a(), r);
        }
    }
}


void rst::rasterizer::draw_elliptic(pos_buf_id pos_buffer, ind_buf_id ind_buffer, rst::Primitive type, int d1, int d2) {
    if (type == rst::Primitive::Elliptic) {
        auto &buf = pos_buf[pos_buffer.pos_id]; // 取出存的圆形相应的数据
        auto &ind = ind_buf[ind_buffer.ind_id];
        Eigen::Matrix4f mvp = projection * view * model;

        for (auto &i : ind) {
            Elliptic e;
            Eigen::Vector4f v[] = {
                    mvp * to_vec4(buf[i[0]], 1.0f)}; // 先画一个圆
            for (auto &vec : v) {
                vec = vec / vec[3]; // 将齐次坐标的最后一位化为1
            }
            for (auto &vert : v) // 将图形放中间放大
            {
                vert.x() = 0.5 * width + (vert.x() + 1.0);
                vert.y() = 0.5 * height + (vert.y() + 1.0);
            }
            e.setVertex(v[0].head<3>());
            Eigen::Vector3f line_color = {255, 255, 0};
            draw_elliptic_line(e.a(), d1, d2);

        }
    }
}


void rst::rasterizer::rasterize_wireframe(const Triangle &t) {
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p) {
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y) {
    return (height - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color) {
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height)
        return;
    auto ind = (height - point.y()) * width + point.x();
    frame_buf[ind] = color;
}
