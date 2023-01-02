// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// add
bool judge(Eigen::Vector3f &p, Eigen::Vector3f &a, Eigen::Vector3f &b) {
    Eigen::Vector3f v = (b - a).cross(p - a);

    return v.z() > 0;
}

static bool insideTriangle(float x, float y, const Triangle* t)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // 获得3个顶点
    Eigen::Vector3f p(x, y, 0.0);
    Eigen::Vector3f a = (t->v)[0];
    Eigen::Vector3f b = (t->v)[1];
    Eigen::Vector3f c = (t->v)[2];

    if (judge(p, a, b) && judge(p, b, c) && judge(p, c, a)) {
        return true;
    }

    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // 转换为齐次坐标
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // bounding box
    float x_min = 1e10;
    float x_max = -1e10;
    float y_min = 1e10;
    float y_max = -1e10;
    for (int i = 0; i < 3; i++) {
        x_min = std::min(x_min, t.v[i](0));
        x_max = std::max(x_max, t.v[i](0));
        y_min = std::min(y_min, t.v[i](1));
        y_max = std::max(y_max, t.v[i](1));
    }
    int x_min_int = int(x_min);
    int x_max_int = int(x_max) + 1;
    int y_min_int = int(y_min);
    int y_max_int = int(y_max) + 1;
    std::cout << x_min_int << " " << x_max_int << " " << width << std::endl;
    std::cout << y_min_int << " " << y_max_int << " " << height << std::endl;
    
    // ssaa
    float x, y;
    // 间距
    float d = 1.0 / n;
    for (int i = x_min_int; i <= x_max_int; i++) {
        for (int j = y_min_int; j <= y_max_int; j++) {
            // 记录super-sample在三角形内的数量
            int cnt = 0;
            // 记录块内的最小深度
            float z_interpolated_min = 1e10;
            // 遍历每个super-sample
            for (int p = 0; p < n; p++) {
                for (int q = 0; q < n; q++) {
                    x = i + (p + 0.5) * d;
                    y = j + (q + 0.5) * d;
                    // 在三角形内则更新cnt
                    if (insideTriangle(x, y, &t)) {
                        cnt++;
                        // 计算插值深度值
                        auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        z_interpolated_min = std::min(z_interpolated_min, z_interpolated);
                    }
                }
            }
            int index = get_index(i, j);
            // 越小越近
            if (z_interpolated_min < depth_buf[index]) {
                // 设置颜色
                Eigen::Vector3f point(i, j, 0);
                set_pixel(point, t.getColor() * cnt / (n * n));
                // 更新深度
                depth_buf[index] = z_interpolated_min;
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h, int n) : width(w), height(h), n(n)
// rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on