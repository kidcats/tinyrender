//
// Created by 1111 on 2020/12/23.
//

#ifndef RASTERIZER_ELLIPTIC_H
#define RASTERIZER_ELLIPTIC_H

#include <eigen3/Eigen/Eigen>

using namespace Eigen;

class Elliptic {
    // 需要长轴长度，短轴长度，中心坐标
    public:
        Vector3f v;
        Vector3f color;
        Vector3f normal;
        float d1,d2;//d1是长轴的一半，d2是短轴的一半
        Elliptic();
        Eigen::Vector3f a() const { return v; }
        float get_d1() const { return d1;}
        float get_d2() const { return d2;}

        void setVertex(Vector3f ver);
        void setColor(float r,float g,float b);
};


#endif //RASTERIZER_ELLIPTIC_H
