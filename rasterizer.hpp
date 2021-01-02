//
// Created by goksu on 4/6/19.
//

#pragma once

#include "Triangle.hpp"
#include "Circle.hpp"
#include "Elliptic.h"
#include <algorithm>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;

namespace rst {
    enum class Buffers {
        Color = 1,
        Depth = 2
    };

//    const std::vector<Eigen::Vector3f> &rasterizer::getFrameBuf() const {
//        return frame_buf;
//    }
//
//    void rasterizer::setFrameBuf(const std::vector<Eigen::Vector3f> &frameBuf) {
//        frame_buf = frameBuf;
//    }

    inline Buffers operator|(Buffers a, Buffers b) // 判断两个buffers是否相同
    {
        return Buffers((int) a | (int) b);
    }

    inline Buffers operator&(Buffers a, Buffers b) {
        return Buffers((int) a & (int) b);
    }

    enum class Primitive // 原始的
    {
        Line,
        Triangle,
        Circle,
        Elliptic,
    };

/*
 * 用于draw函数,作为缓冲区id
 * For the curious : The draw function takes two buffer id's as its arguments.
 * These two structs make sure that if you mix up with their orders, the
 * compiler won't compile it. Aka : Type safety
 * */
    struct pos_buf_id {
        int pos_id = 0;
    };

    struct ind_buf_id {
        int ind_id = 0;
    };

    class rasterizer {
    public:
        rasterizer(int w, int h); // 宽高
        pos_buf_id load_positions(const std::vector<Eigen::Vector3f> &positions); // 位置
        ind_buf_id load_indices(const std::vector<Eigen::Vector3i> &indices); // 指数

        void set_model(const Eigen::Matrix4f &m);

        void set_view(const Eigen::Matrix4f &v);

        void set_projection(const Eigen::Matrix4f &p);

        // 设置像素点颜色
        void set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color);

        void clear(Buffers buff);

        // 位置, 指数, 类型
        void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type);

        void draw_circle(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type, int r);

        void draw_elliptic(pos_buf_id pos_buffer, ind_buf_id ind_buffer, rst::Primitive type, int d1, int d2);

        std::vector<Eigen::Vector3f> &frame_buffer() { return frame_buf; }

        const std::vector<Eigen::Vector3f> &getFrameBuf() const;

        void setFrameBuf(const std::vector<Eigen::Vector3f> &frameBuf);

    private:
        void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);

        void draw_circle_line(Eigen::Vector3f point, int r);

        void draw_elliptic_line(Vector3f v, int d1, int d2);

        void rasterize_wireframe(const Triangle &t);

    private:
        Eigen::Matrix4f model;
        Eigen::Matrix4f view;
        Eigen::Matrix4f projection;

        std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
        std::map<int, std::vector<Eigen::Vector3i>> ind_buf;

        std::vector<Eigen::Vector3f> frame_buf;
        std::vector<float> depth_buf;

        int get_index(int x, int y);

        int width, height;

        int next_id = 0;

        int get_next_id() { return next_id++; }
    };
} // namespace rst
