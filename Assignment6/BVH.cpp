#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    // root = recursiveBuild(primitives);
    root = recursiveBuildSVH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuildSVH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildSVH(std::vector{objects[0]});
        node->right = recursiveBuildSVH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        // 原则
        // 找到最宽松的轴进行划分
        Bounds3 centroidBounds;
        Bounds3 maxBounds;
        for (int i = 0; i < objects.size(); ++i) {
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
            maxBounds = Union(maxBounds, objects[i]->getBounds());
        }
        float total_size = maxBounds.SurfaceArea();

        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        // 策略
        float min_cos = INFINITY;
        // 计算面积
        int n = objects.size();
        // 从左往右前i个
        std::vector<float> left_size(n + 1, 0);
        // 从右往左前i个
        std::vector<float> right_size(n + 1, 0);
        std::vector<float> score(n + 1, 0);
        Bounds3 left_bound;
        Bounds3 right_bound;
        for (int i = 1; i <= n; i++) {
            left_bound = Union(left_bound, objects[i - 1]->getBounds());
            left_size[i] = left_bound.SurfaceArea() / total_size;
        }
        for (int i = 1; i <= n; i++) {
            right_bound = Union(right_bound, objects[n - i]->getBounds());
            right_size[i] = right_bound.SurfaceArea() / total_size;
        }
        // 计算总分
        for (int i = 0; i <= n; i++) {
            score[i] = i * left_size[i] + (n - i) * right_size[n - i];
        }
        // 找到分数最小的位置
        float s = INFINITY;
        int index = -1;
        for (int i = 0; i <= n; i++) {
            if (score[i] < s) {
                s = score[i];
                index = i;
            }
        }
        // 保证划分后都有元素
        if (index == n) {
            index--;
        } else if (index == 0) {
            index++;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + index;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildSVH(leftshapes);
        node->right = recursiveBuildSVH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection isect;
    Vector3f invDir = ray.direction_inv;
    std::array<int, 3> dirIsNeg {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    // 相交
    if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
        if (node == NULL) {
            return isect;
        }
        if (node->left == NULL && node->right == NULL) {
            isect = node->object->getIntersection(ray);
            return isect;
        }
        Intersection isect_left = getIntersection(node->left, ray);
        Intersection isect_right = getIntersection(node->right, ray);
        if (isect_left.distance < isect_right.distance) {
            isect = isect_left;
        } else {
            isect = isect_right;
        }
    }

    return isect;
}