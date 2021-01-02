//
// Created by 1111 on 2020/12/23.
//

#include "Elliptic.h"
Elliptic::Elliptic() {
    v << 0,0,0;
    color << 0.0,0.0,0.0;
}

void Elliptic::setVertex(Vector3f ver) {
    v = ver;
}

void Elliptic::setColor(float r, float g, float b) {
    if ((r < 0.0 || r > 255.)|| (g < 0.0 || g > 255.)||(b < 0.0 || b > 255.)){
        throw std::runtime_error("Invalid color values");
    }
    color = Vector3f ((float )r/255,(float) g/255,(float) b/255);
    return;
}
