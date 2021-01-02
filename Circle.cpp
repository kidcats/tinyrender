//
// Created by cats kid on 2020/12/14.
//

#include "Circle.hpp"
#include <stdexcept>

Circle::Circle() {
    v << 0,0,0;
    color << 0,0,0;
}

void Circle::setVertex(Vector3f ver) {
    v = ver;
}

void Circle::setNormal(Vector3f n) {
    normal = n;
}

void Circle::setColor(float r, float g, float b) {
    if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) ||
        (b > 255.)) {
        throw std::runtime_error("Invalid color values");
    }
    color = Vector3f ((float) r / 255,(float) g / 255,(float) b / 255);
}

//std::array<Vector4f, 3> Circle::toVector4() const
//{
//    std::array<Vector4f, 3> res;
//    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) {
//        return Vector4f(vec.x(), vec.y(), vec.z(), 1.f);
//    });
//    return res;
//}