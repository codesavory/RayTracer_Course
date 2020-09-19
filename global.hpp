#pragma once
#include <iostream>
#include <cmath>
#include <random>

#define M_PI 3.14159265358979323846

extern const float  EPSILON;
const float kInfinity = std::numeric_limits<float>::max();

inline float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }

inline void clampVec(const float& lo, const float& hi, Vector3f& v)
{
    v.x = clamp(lo, hi, v.x);
    v.y = clamp(lo, hi, v.y);
    v.z = clamp(lo, hi, v.z);
}

inline  bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0) return false;
    else if (discr == 0) x0 = x1 = - 0.5 * b / a;
    else {
        float q = (b > 0) ?
                  -0.5 * (b + sqrt(discr)) :
                  -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1) std::swap(x0, x1);
    return true;
}

inline float get_random_float()
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]

    return dist(rng);
}

inline void UpdateProgress(float progress)
{
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
};

/*float attenuate(float d)
{
    float c1 = 0.25;
    float c2 = 0.1f;
    float c3 = 0.01f;

    float den = c1 + c2 * d + c3 * (d * d);
    return std::min(1.0f, 1.0f / den);
}*/

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

inline double random_float() {
    static std::uniform_real_distribution<float> distribution(0.0, 1.0);
    static std::mt19937 generator;
    return distribution(generator);
}

inline double random_float(float min, float max) {
    // Returns a random real in [min,max).
    return min + (max - min) * random_float();
}