#pragma once
#pragma once
#include "global.hpp"
#include "Ray.hpp"
#include <cmath>
#include <iostream>


class camera {
public:
    camera(
        point3 lookfrom,
        point3 lookat,
        vec3   vup,
        double vfov, // vertical field-of-view in degrees
        double aspect_ratio
    ) {
        auto theta = deg2rad(vfov);
        auto h = tan(theta / 2);
        auto viewport_height = 2.0 * h;
        auto viewport_width = aspect_ratio * viewport_height;

        vup = -1 * vup;

        auto w = normalize((normalize(lookfrom) - lookat));
        auto u = normalize(crossProduct(vup, w));
        auto v = crossProduct(w, u);

        origin = lookfrom;
        horizontal = viewport_width * u;
        vertical = viewport_height * v;
        lower_left_corner = origin - horizontal / 2 - vertical / 2 - w;
    }

    C_ray get_ray(double s, double t) const {
        return C_ray(origin, lower_left_corner + s * horizontal + t * vertical - origin);
    }

private:
    point3 origin;
    point3 lower_left_corner;
    vec3 horizontal;
    vec3 vertical;
};