#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
Model *model = NULL;
const int width = 800;
const int height = 800;
/*
a,b,c是顶点坐标 ，p是输入点坐标
*/
Vec3f barycentric(Vec2i a, Vec2i b, Vec2i c, Vec2i p)
{
    float u = (float)((p.y - b.y) * (c.x - b.x) - (p.x - b.x) * (c.y - b.y)) /
              ((a.y - b.y) * (c.x - b.x) - (a.x - b.x) * (c.y - b.y));
    float v = (float)((p.y - c.y) * (a.x - c.x) - (p.x - c.x) * (a.y - c.y)) /
              ((b.y - c.y) * (a.x - c.x) - (b.x - c.x) * (a.y - c.y));
    return Vec3f(u, v, 1 - u - v);
}

void line(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color)
{
    bool steep = false;
    if (std::abs(p0.x - p1.x) < std::abs(p0.y - p1.y))
    {
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }
    if (p0.x > p1.x)
    {
        std::swap(p0, p1);
    }
    for (int x = p0.x; x <= p1.x; x++)
    {
        float t = (x - p0.x) / (float)(p1.x - p0.x);
        int y = p0.y * (1. - t) + p1.y * t;
        if (steep)
        {
            image.set(y, x, color);
        }
        else
        {
            image.set(x, y, color);
        }
    }
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color)
{
    if (t0.y == t1.y && t0.y == t2.y)
        return;
    if (t0.y > t1.y)
        std::swap(t0, t1);
    if (t0.y > t2.y)
        std::swap(t0, t2);
    if (t1.y > t2.y)
        std::swap(t2, t1); // 保证是按照t0<t1<t2的顺序排列
    int total_height = t2.y - t0.y;
    // 先找到三角形的上下左右边界
    int total_right = std::max(std::max(t0.x, t1.x), t2.x);
    int total_left = std::min(std::min(t0.x, t1.x), t2.x);
    // 利用重心坐标判断点是否在三角形内部
    for (int i = total_left; i < total_right; i++)
    {
        for (int j = t0.y; j < t2.y; j++)
        {
            // 先求出点对应的重心坐标
            Vec2i point = {i, j};
            auto bary = barycentric(t0, t1, t2, point);
            bool in_triangle = bary.x >= 0 && bary.y >= 0 && bary.z >= 0;
            if (!in_triangle)
            {
                continue;
            }
            else
            {
                image.set(i, j, color);
            }
        }
    }
}

int main(int argc, char **argv)
{

    if (argc == 2)
    {
        model = new Model(argv[1]);
    }
    else
    {
        model = new Model("obj/african_head.obj");
    }
    TGAImage image(width, height, TGAImage::RGB);
    Vec3f light_dir(0, 0, -1);

    for (int i = 0; i < model->nfaces(); i++)
    {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; j++)
        {
            Vec3f v = model->vert(face[j]);
            screen_coords[j] = Vec2i((v.x + 1.) * width / 2., (v.y + 1.) * height / 2.);
            world_coords[j] = v;
        }
        Vec3f n = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
        n.normalize();
        float intensity = n * light_dir;
        if (intensity > 0)
        {
            triangle(screen_coords[0], screen_coords[1], screen_coords[2], image, TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));
        }
    }
    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
