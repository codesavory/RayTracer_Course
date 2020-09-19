#pragma once
#include "Vector.hpp"
#include "Material.hpp"


class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened = false;
        coords = Vector3f();
        normal = Vector3f();
        distance = std::numeric_limits<double>::max();
        obj = nullptr;
        m = Material();
    }
    bool happened;
    Vector3f coords;
    Vector3f normal;
    double distance;
    Object* obj;
    Material m;
};
