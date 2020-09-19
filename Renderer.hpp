#pragma once
#include "Eigen/Eigen"
#include "Scene.hpp"
#include "scene_io.h"
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);
    void Render(const Scene& scene, CameraIO* cam);
private:
};
