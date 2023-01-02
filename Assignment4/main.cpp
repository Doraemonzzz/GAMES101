#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    int n = control_points.size();
    if (n == 1) {
        return control_points[0];
    }
    std::vector<cv::Point2f> res;
    for (int i = 0; i < n - 1; i++) {
        auto point = t * control_points[i] + (1 - t) * control_points[i + 1];
        res.push_back(point);
    }

    return recursive_bezier(res, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window, bool flag) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = recursive_bezier(control_points, t);
        int x = point.x;
        int y = point.y;
        if (flag) {
            // 反走样
            int y_min = int(point.y);
            int y_max = y_min + 1;
            int x_min = int(point.x);
            int x_max = x_min + 1;
            // 找到最近的位置
            if (point.y < y_min + 0.5) {
                y = y_min;
            } else {
                y = y_max;
            }
            if (point.x < x_min + 0.5) {
                x = x_min;
            } else {
                x = x_max;
            }
        }
        // 该函数使用的参数为int
        window.at<cv::Vec3b>(y, x)[1] = 255;
    }
}

int main(int argc, const char** argv) 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    std::string file_name = "my_bezier_curve.png";
    bool flag = false;
    if (argc == 2) {
        flag = true;
        file_name = std::string(argv[1]);
    }
    std::cout << file_name << std::endl;

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window, flag);

            cv::imshow("Bezier Curve", window);
            cv::imwrite(file_name.c_str(), window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
