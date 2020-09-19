#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include "Triangle.hpp"
#include <cassert>
#include <array>

bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1,
                          const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;
    Vector3f pvec = crossProduct(dir, edge2);
    float det = dotProduct(edge1, pvec);
    if (det == 0 || det < 0)
        return false;

    Vector3f tvec = orig - v0;
    u = dotProduct(tvec, pvec);
    if (u < 0 || u > det)
        return false;

    Vector3f qvec = crossProduct(tvec, edge1);
    v = dotProduct(dir, qvec);
    if (v < 0 || u + v > det)
        return false;

    float invDet = 1 / det;

    tnear = dotProduct(edge2, qvec) * invDet;
    if (tnear < 0)
        return false;

    tnear = dotProduct(edge2, qvec) * invDet;
    u *= invDet;
    v *= invDet;

    return true;
}

/*class Triangle : public Object
{
public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Vector3f t0, t1, t2; // texture coords
    Vector3f normal;
    std::shared_ptr<Material> m;

    NormType normType;
    std::vector<Vec3f> normals;


    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, std::shared_ptr<Material> _m)
        : v0(_v0), v1(_v1), v2(_v2), m(_m)
    {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
    }

    bool intersect(const Ray& ray) override;
    bool intersect(const Ray& ray, float& tnear,
                   uint32_t& index) const override;
    Intersection getIntersection(Ray ray) override;
    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                              const uint32_t& index, const Vector2f& uv,
                              Vector3f& N, Vector2f& st) const override
    {
        N = normal;
        //        throw std::runtime_error("triangle::getSurfaceProperties not
        //        implemented.");
    }
    Vector3f evalDiffuseColor(const Vector2f&) const override;
    Bounds3 getBounds() override;
};*/

class Triangle : public Object {
public:
    std::array<Vector3f, 3> vertices;
    std::array<Vector3f, 3> normals;
    std::array<Vector2f, 3> stC;
    std::array<Material, 3> materials;

    NormType normType;
    MaterialBinding materialBinding;

    Triangle(std::array<Vector3f, 3> vertices, std::array<Vector3f, 3> normals,
        std::array<Vector2f, 3> stC, std::array<Material, 3> materials, 
        NormType normType, MaterialBinding materialBinding) :

        vertices(vertices), 
        normals(normals), 
        stC(stC), 
        materials(materials), 
        normType(normType),
        materialBinding(materialBinding)
    {
    }

    bool intersect(const Ray& ray) override;
    bool intersect(const Ray& ray, float& tnear,
        uint32_t& index) const override;
    Intersection getIntersection(Ray ray) override;
    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
        const uint32_t& index, const Vector2f& uv,
        Vector3f& N, Vector2f& st) const override
    {
        //N = normal;
        //        throw std::runtime_error("triangle::getSurfaceProperties not
        //        implemented.");
    }
    Vector3f evalDiffuseColor(const Vector2f&) const override;
    Bounds3 getBounds() override;

    Vec3f interpolateNormal(float u, float v, float w) const
    {
        if (normType == PER_FACE_NORMAL)
        {
            const Vec3f& v0 = vertices[0];
            const Vec3f& v1 = vertices[1];
            const Vec3f& v2 = vertices[2];


            Vec3f N;
            Vec3f e0 = v1 - v0;
            e0 = normalize(e0);
            Vec3f e1 = v2 - v1;
            e1 = normalize(e1);
            N = crossProduct(e0, e1);
            N = normalize(N);
            return N;
        }
        else
        {
            const Vec3f& n0 = normals[0];
            const Vec3f& n1 = normals[1];
            const Vec3f& n2 = normals[2];
            return normalize(u * n0 + v * n1 + w * n2);
        }
    }

    Material interpolateMaterial(float u, float v, float w) const
    {
        if (materialBinding == PER_OBJECT_MATERIAL)
        {
            return materials[0];
        }
        else
        {
            const Material& m0 = materials[0];
            const Material& m1 = materials[1];
            const Material& m2 = materials[2];

            Material m3 = Material::interpolateByBaryCentric(m0, m1, m2, u, v, w);
            return m3;
        }
    }


    


};

inline bool Triangle::intersect(const Ray& ray) { return true; }
inline bool Triangle::intersect(const Ray& ray, float& tnear,
    uint32_t& index) const
{
    return false;
}

inline Bounds3 Triangle::getBounds() {
    const Vec3f& v0 = vertices[0];
    const Vec3f& v1 = vertices[1];
    const Vec3f& v2 = vertices[2];
    return Union(Bounds3(v0, v1), v2); 
}

inline Intersection Triangle::getIntersection(Ray ray)
{
    Intersection inter;
    float u, v, t = 0;
    const Vec3f& v0 = vertices[0];
    const Vec3f& v1 = vertices[1];
    const Vec3f& v2 = vertices[2];

    // TODO find ray triangle intersection, and fill the intersection object
    if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t, u, v))
    {
        float w = 1 - u - v;
        auto N = interpolateNormal(w, u, v);
        auto M = interpolateMaterial(w, u, v);

        inter.obj = dynamic_cast<Object*>(this);
        inter.m = M;
        inter.happened = true;
        inter.normal = N;
        inter.coords = t * ray.direction + ray.origin;
        inter.distance = t;
    }

    return inter;
}

inline Vector3f Triangle::evalDiffuseColor(const Vector2f&) const
{
    return Vector3f(0.5, 0.5, 0.5);
    //return m->diffuseColor;
}





class MeshTriangle : public Object
{
public:
    /*MeshTriangle(
        std::vector<Vec3f> points, std::vector<uint32_t> indices, float num_poly, 
        std::vector<Vec2f> stVec, 
        std::vector<Material> materials, std::vector<uint32_t> materialIds)*/

        MeshTriangle(
            const std::vector<Vec3f> points,
            const std::vector<Vec3f> normals,
            const std::vector<uint32_t> indices,
            const uint32_t& num_poly,
            const std::vector<Vec2f> stVec,
            const std::vector<Material> materials,
            const std::vector<uint32_t> materialIds,
            const NormType normType, const MaterialBinding materialBinding
        )
    {
        Vector3f min_vert = Vector3f{ std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity() };
        Vector3f max_vert = Vector3f{ -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity() };


        for (int i = 0; i < indices.size(); i += 3) {
            std::array<Vector3f, 3> v;
            std::array<Vector3f, 3> n;
            std::array<Vector2f, 3> stC;
            std::array<Material, 3> m;

            for (int j = 0; j < 3; j++) {
                auto v_id = indices[i + j];
                auto vert = points[v_id];

                v[j] = points[v_id];
                n[j] = normals[v_id];
                stC[j] = stVec[v_id];
                
                auto mId = materialIds[v_id];
                auto material = materials[mId];
                m[j] = material;

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                    std::min(min_vert.y, vert.y),
                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                    std::max(max_vert.y, vert.y),
                    std::max(max_vert.z, vert.z));
            }

            Triangle t(v, n, stC, m, normType, materialBinding);
            triangles.push_back(t);
        }


        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object*> ptrs;
        for (auto& tri : triangles)
            ptrs.push_back(&tri);

        bvh = new BVHAccel(ptrs);
    }

    bool intersect(const Ray& ray) { return true; }

    bool intersect(const Ray& ray, float& tnear, uint32_t& index) const
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k) {
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t,
                                     u, v) &&
                t < tnear) {
                tnear = t;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    Bounds3 getBounds() { return bounding_box; }

    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                              const uint32_t& index, const Vector2f& uv,
                              Vector3f& N, Vector2f& st) const
    {
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f& st) const
    {
        float scale = 5;
        float pattern =
            (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031),
                    Vector3f(0.937, 0.937, 0.231), pattern);
    }

    Intersection getIntersection(Ray ray)
    {
        Intersection intersec;

        if (bvh) {
            intersec = bvh->Intersect(ray);
        }

        return intersec;
    }

    Bounds3 bounding_box;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    NormType normType;
    std::vector<Vec3f> normals;

    std::vector<Triangle> triangles;

    BVHAccel* bvh;
};


