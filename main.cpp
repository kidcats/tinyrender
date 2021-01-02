#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

/**
 * @brief Get the view matrix object 视图变换
 * 将x to (g*t) ,y to t , z to -g进行旋转变换然后再求逆矩阵
 * 
 * @param eye_pos 
 * @return Eigen::Matrix4f 
 */
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotate(4, 4); // z-axis rotation
    float radian = rotation_angle / 180.0 * MY_PI;
    rotate << cos(radian), -sin(radian), 0, 0,
        sin(radian), cos(radian), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    model = rotate * model;

    return model;
}

/**
 * @brief Get the projection matrix object 投影矩阵,有正交投影和透视投影两个,先透视后正交
 * 
 * @param eye_fov 决定上下的范围,即y
 * @param aspect_ratio 长宽比
 * @param zNear 近处的长度
 * @param zFar 远处的长度
 * @return Eigen::Matrix4f 
 */
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f M_persp2ortho(4, 4);
    Eigen::Matrix4f M_ortho_scale(4, 4);
    Eigen::Matrix4f M_ortho_trans(4, 4);
    //已更正
    float angle = eye_fov * MY_PI / 180.0;
    float height = zNear * tan(angle) * 2; // 挤压后的高(y轴)
    float width = height * aspect_ratio;   // 挤压后的宽(x轴)

    auto t = -zNear * tan(angle / 2); // 上截面
    auto r = t * aspect_ratio;        //右截面
    auto l = -r;                      // 左截面
    auto b = -t;                      // 下截面
    // 透视矩阵"挤压"
    M_persp2ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;
    // 正交矩阵-缩放
    M_ortho_scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;
    // 正交矩阵-平移
    M_ortho_trans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;
    Eigen::Matrix4f M_ortho = M_ortho_scale * M_ortho_trans;
    projection = M_ortho * M_persp2ortho * projection;

    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700); // 当不是700的时候会有很奇怪的错误,一会记得看

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, 0}, {0, 2, 0}, {-2, 0, 0}};
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    // 如果渲染一个凸多边形的画,需要的三角形个数是n-2所以pos_in = n-2
    std::vector<Eigen::Vector3f> vec{{2, 0, 0}, {0, 2, 0}, {-2, 0, 0}, {0, -2, 0}};

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        // r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        //        r.draw_circle(pos_id,ind_id,rst::Primitive::Circle,100);
        // 化凸多边形
    
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));


        // r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        //         r.draw_circle(pos_id,ind_id,rst::Primitive::Circle,200);
        // r.draw_elliptic(pos_id,ind_id,rst::Primitive::Elliptic,200,100);
        for(int i = 2;i < vec.size(); i++){
            auto pos1 = r.load_positions({vec[0],vec[i-1],vec[i]});

            auto ind1 = r.load_indices(ind);
            r.draw(pos1,ind1,rst::Primitive::Triangle);
        }
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
