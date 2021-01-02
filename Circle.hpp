//
// Created by cats kid on 2020/12/14.
//

#ifndef RASTERIZER_CIRCLE_HPP
#define RASTERIZER_CIRCLE_HPP

#include <eigen3/Eigen/Eigen>

using namespace Eigen;

class Circle {
public:
    Vector3f v;// 中心点坐标
    int r; // 半径长度
    Vector3f color;// 圆的颜色
    Vector3f normal;// 标准化后的坐标

    Circle();

    Eigen::Vector3f a() const { return v; }

    void setVertex(Vector3f ver); /*set i-th vertex coordinates */
    void setNormal(Vector3f n);   /*set i-th vertex normal vector*/
    void setColor(float r, float g, float b); /*set i-th vertex color*/

    std::array<Vector4f,3> toVector4() const;
};


#endif //RASTERIZER_CIRCLE_HPP
