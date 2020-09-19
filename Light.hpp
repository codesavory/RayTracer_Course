#pragma once

#include "Vector.hpp"
#include "scene_io.h"

class Light
{
public:
    Light(const Vector3f &p, const Vector3f &d, const Vector3f &i, LightType t) : 
        position(p), direction(d), intensity(i), type(t) {}
    virtual ~Light() = default;
    Vector3f position;
    Vector3f direction;
    Vector3f intensity;
    LightType type;
};
