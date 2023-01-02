//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Material.hpp"
#include "global.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // 光线和场景的交点
    Intersection t1 = intersect(ray);
    // 没有交点
    if (!t1.happened) {
        return Vector3f();
    }
    // 如果物体发光
    if (t1.emit.norm() > 0) {
        // 物体发光部分只统计一次
        if (depth == 0) {
            return t1.emit;
        } else {
            return Vector3f();
        }
    }
    // 交点位置
    Vector3f p = t1.coords;
    // 场景法线
    Vector3f N = (t1.normal).normalized();
    // 入射方向
    Vector3f wo = (-ray.direction).normalized();
    // 材质
    Material *m1 = t1.m;
    // samplelight
    Intersection inter;
    float pdf_light;
    sampleLight(inter, pdf_light);
    // 光源位置
    Vector3f x = inter.coords;
    // 出射方向
    Vector3f ws = (x - p).normalized();
    // 光源法线
    Vector3f NN = (inter.normal).normalized();
    // Radiance Li
    Vector3f emit = inter.emit;
    // 距离
    float dist = (x - p).norm();
    // shoot a ray from p to x
    Ray px = Ray(p, ws);
    Intersection px_inter = intersect(px);
    // 交点位置
    Vector3f x_inter = px_inter.coords;
    // If the ray is not blocked in the middle
    Vector3f L_dir = Vector3f();
    if (px_inter.happened && ((x_inter - x).norm() < 1e-3)) {
        // 不要使用pdf_light + eps, 否则会全黑
        L_dir = emit * m1->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (dist * dist) / pdf_light;
    }
    Vector3f L_indir = Vector3f();
    // Test Russian Roulette with probability RussianRoulette
    if (get_random_float() < RussianRoulette) {
        Vector3f wi = m1->sample(wo, N).normalized();
        // Trace a ray r(p, wi)
        Ray ray_new(p, wi);
        // If ray r hit a non -emitting object at q
        // 光线和场景的交点
        Intersection t2 = intersect(ray_new);
        Material *m2 = t1.m;
        float pdf = m1->pdf(wo, wi, N);
        L_indir = castRay(ray_new, depth + 1) * m2->eval(wo, wi, N) * dotProduct(wi, N) / pdf / RussianRoulette;
    }

    return L_dir + L_indir;
}