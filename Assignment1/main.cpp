#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotate = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f tmp;
    tmp << 0, -axis.z(), axis.y(), 
           axis.z(), 0, -axis.x(), 
           -axis.y(), axis.x(), 0;
    float alpha = angle / 180 * MY_PI;
    rotate += std::cos(alpha) * Eigen::Matrix3f::Identity() + 
              (1 - std::sin(alpha)) * axis * axis.transpose() + \
              std::sin(alpha) * tmp;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3;j ++) {
            model(i, j) = rotate(i, j);
        }
    }
    
    return model;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Vector3f z(0, 0, 1);
    Eigen::Matrix4f model = get_rotation(z, rotation_angle);

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    // persp
    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Zero();
    // 保证为负数
    float n = zNear;
    float f = zFar;
    float A = n + f;
    float B = - n * f;
    persp2ortho << n, 0, 0, 0,
                   0, n, 0, 0,
                   0, 0, A, B,
                   0, 0, 1, 0;

    // orth
    Eigen::Matrix4f scale = Eigen::Matrix4f::Zero();
    Eigen::Matrix4f tran = Eigen::Matrix4f::Identity();
    // 不加-图形为反向
    float t = -std::tan(eye_fov / 180 * MY_PI / 2) * std::abs(n);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;
    scale << 2 / (r - l), 0, 0, 0,
             0, 2 / (t - b), 0, 0,
             0, 0, 2 / (n - f), 0,
             0, 0, 0, 1;
    tran << 1, 0, 0, -(r + l) / 2,
            0, 1, 0, -(t + b) / 2,
            0, 0, 1, -(n + f) / 2,
            0, 0, 0, 1;
            
    // final result
    projection = scale * tran * persp2ortho;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
