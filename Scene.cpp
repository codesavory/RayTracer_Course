#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
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

// Implementation of the Whitted-syle light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refracton direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refractin depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is duffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
/*Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }
    Intersection intersection = Scene::intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
//    float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;
    if(intersection.happened) {

        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
//        Vector3f tmp = hitPoint;
        switch (m->getType()) {
            case REFLECTION_AND_REFRACTION:
            {
                Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
                Vector3f refractionDirection = normalize(refract(ray.direction, N, m->ior));
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint - N * EPSILON :
                                             hitPoint + N * EPSILON;
                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * EPSILON :
                                             hitPoint + N * EPSILON;
                Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
                float kr;
                fresnel(ray.direction, N, m->ior, kr);
                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
            case REFLECTION:
            {
                float kr;
                fresnel(ray.direction, N, m->ior, kr);
                Vector3f reflectionDirection = reflect(ray.direction, N);
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint + N * EPSILON :
                                             hitPoint - N * EPSILON;
                hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection),depth + 1) * kr;
                break;
            }
            default:
            {
                // [comment]
                // We use the Phong illumation model int the default case. The phong model
                // is composed of a diffuse and a specular reflection component.
                // [/comment]
                Vector3f lightAmt(0.0f), specularColor(0.0f);
                Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ?
                                           hitPoint + N * EPSILON :
                                           hitPoint - N * EPSILON;
                // [comment]
                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                // [/comment]
                for (uint32_t i = 0; i < get_lights().size(); ++i)
                {
                    auto area_ptr = dynamic_cast<AreaLight*>(this->get_lights()[i].get());
                    if (area_ptr)
                    {
                        
                    }
                    else
                    {
                        Vector3f lightDir = get_lights()[i]->position - hitPoint;
                        // square of the distance between hitPoint and the light
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        lightDir = normalize(lightDir);
                        float LdotN = std::max(0.f, dotProduct(lightDir, N));
                        Object *shadowHitObject = nullptr;
                        float tNearShadow = kInfinity;
                        // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                        bool inShadow = bvh->Intersect(Ray(shadowPointOrig, lightDir)).happened;
                        lightAmt += (1 - inShadow) * get_lights()[i]->intensity * LdotN;
                        Vector3f reflectionDirection = reflect(-lightDir, N);
                        specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)),
                                              m->specularExponent) * get_lights()[i]->intensity;
                    }
                }
                auto c = hitObject->evalDiffuseColor(st);
                hitColor = lightAmt * ( c* m->Kd + specularColor * m->Ks);
                clampVec(0.0f, 1.0f, hitColor);
                break;
            }
        }
    }

    return hitColor;
}*/


float attenuate(float d)
{
    float c1 = 0.25;
    float c2 = 0.01f;
    float c3 = 0.001f;

    float den = c1 + c2 * d + c3 * (d * d);
    return std::min(1.0f, 1.0f / den);
}

Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    if (depth > this->maxDepth) {
        return Vector3f(0.0, 0.0, 0.0);
    }

    Intersection intersection = Scene::intersect(ray);
    Material m = intersection.m;
    Object* hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
    //    float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;
    if (intersection.happened) {

        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates
        //hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);

        // ambient light part
        Vec3f diffcol = m.ka * m.diffuseColor;
        Vec3f ambientLight = (1 - m.ktran) * diffcol;


        // direct illumination part
        Vec3f directIllum(0.0, 0.0, 0.0);
        for (uint32_t i = 0; i < lights.size(); ++i) {

            // check if point in shadow or not. ignore transparency for now
            Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ?
                hitPoint + N * EPSILON :
                hitPoint - N * EPSILON;

            Vec3f lightDir;
            float lightDistance;
            if (lights[i]->type == DIRECTIONAL_LIGHT)
            {
                lightDir = -1 * lights[i]->direction;
                lightDistance = kInfinity;
            }
            else
            {
                lightDir = lights[i]->position - hitPoint;
                lightDistance = dotProduct(lightDir, lightDir);
                lightDir = normalize(lightDir);
            }
            


            Object* shadowHitObject = nullptr;
            float tNearShadow = kInfinity;
            uint32_t shadowI;
            // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
            //bool inShadow = trace(shadowPointOrig, lightDir, objects, tNearShadow,
            //    shadowI, uv, &shadowHitObject) && tNearShadow * tNearShadow < lightDistance;
            auto sIntersect = bvh->Intersect(Ray(shadowPointOrig, lightDir));
            bool inShadow = sIntersect.happened;
            if(lights[i]->type == POINT_LIGHT)
                inShadow = inShadow && sIntersect.distance* sIntersect.distance < lightDistance;

            // The following should ideally be done recursively. For now just doing 1 bounce
            Vec3f S(1.0f, 1.0f, 1.0f);
            /*if (inShadow)
            {
                Vec3f s_N; // normal
                Vec2f s_st; // st coordinates
                Material s_mat;
                Vec3f shadowHitPoint = shadowPointOrig + lightDir * tNearShadow;
                shadowHitObject->getSurfaceProperties(shadowHitPoint, lightDir, shadowI, uv, s_N, s_st, s_mat);

                Vec3f c = shadowHitObject->evalDiffuseColor(s_st);
                auto maxC = c.maxCoeff();
                c /= maxC;
                S = shadowHitObject->ktran * S.cwiseProduct(c);

            }*/

            // Shadow factor; ignore transparent objects
            S = Vec3f(1.0 - inShadow, 1.0 - inShadow, 1.0 - inShadow);
            Vec3f lightIntensity = lights[i]->intensity;
            float attenuateF = 1.0f;
            if (lights[i]->type == DIRECTIONAL_LIGHT)
                attenuateF = 1.0f;
            else
                attenuateF = attenuate(lightDistance);


            // diffuse 
            Vec3f diffuseColor = m.diffuseColor;
            

            if (lights[i]->type == DIRECTIONAL_LIGHT)
            {
                lightDir =  -1*lights[i]->direction;
            }
            else
            {
                lightDir = lights[i]->position - hitPoint;
                lightDir = normalize(lightDir);
            }

            float LdotN = std::max(0.0f, dotProduct(lightDir, N));
            Vec3f diffuseComp = (1 - m.ktran) * diffuseColor * LdotN;

            // specular
            Vec3f reflectionDirection = reflect(-lightDir, N);
            Vec3f specularColor = powf(
                std::max(0.0f, dotProduct(-reflectionDirection, ray.direction)),
                m.specularExponent * 128) * Vec3f(1.0f, 1.0f, 1.0f);
            Vec3f specularComp = m.ks * specularColor;

            // skipping attenuation for now since it makes everything dark
            directIllum += 1.0f * S * lightIntensity * (diffuseComp + specularComp);
        }


        // reflection part
        Vec3f reflectionDirection = reflect(ray.direction, N);
        reflectionDirection = normalize(reflectionDirection);

        Vec3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
            Vec3f(hitPoint - N * EPSILON) :
            Vec3f(hitPoint + N * EPSILON);

        Vec3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
        reflectionColor = reflectionColor * m.ks;

        hitColor = ambientLight + directIllum + reflectionColor;
        clampVec(0.0f, 1.0f, hitColor);
    }

    return hitColor;
}