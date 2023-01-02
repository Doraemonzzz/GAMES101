//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f img2Color(int v, int u) {
        auto color = image_data.at<cv::Vec3b>(v, u);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;
        int u_img_min = (int) u_img;
        int u_img_max = u_img_min + 1;
        int v_img_min = (int) v_img;
        int v_img_max = v_img_min + 1;
        float s = u_img - u_img_min;
        float t = v_img - v_img_min;

        auto c00 = img2Color(v_img_min, u_img_min);
        auto c10 = img2Color(v_img_min, u_img_max);
        auto c01 = img2Color(v_img_max, u_img_min);
        auto c11 = img2Color(v_img_max, u_img_max);

        auto c0 = lerp(s, c00, c10);
        auto c1 = lerp(s, c01, c11);
        auto color = lerp(t, c0, c1);
        
        return color;
    }

    Eigen::Vector3f lerp(float s, Eigen::Vector3f a, Eigen::Vector3f b)
    {
        return a + s * (b - a);
    }

};
#endif //RASTERIZER_TEXTURE_H
