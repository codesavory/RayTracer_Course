#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include "scene_io.h"
#include "camera.hpp"

// ******* The following lines have to be added for stb_image_write to
// compile properly
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define M_PI 3.14159265358979323846  /* pi */



const float EPSILON = 0.00001;

Eigen::Matrix4f lookAt(Eigen::Vector3f& from, Eigen::Vector3f& to,
    Eigen::Vector3f& orthoUp)
{
    //Vec3f tmp = Vec3f(0, 1, 0);
    to = to.normalized();
    orthoUp = -1 * orthoUp.normalized();

    /*Vec3f forward = (from.normalized() - to).normalized();
    Vec3f right = tmp.normalized().cross(forward);
    Vec3f up = forward.cross(right);*/

    Eigen::Vector3f forward = (from.normalized() - to).normalized();
    Eigen::Vector3f right = forward.cross(orthoUp);
    Eigen::Vector3f up = orthoUp;

    Eigen::Matrix4f camToWorld = Eigen::Matrix4f::Identity();

    camToWorld(0, 0) = right.x();
    camToWorld(0, 1) = right.y();
    camToWorld(0, 2) = right.z();
    camToWorld(1, 0) = up.x();
    camToWorld(1, 1) = up.y();
    camToWorld(1, 2) = up.z();
    camToWorld(2, 0) = forward.x();
    camToWorld(2, 1) = forward.y();
    camToWorld(2, 2) = forward.z();

    camToWorld(3, 0) = from.x();
    camToWorld(3, 1) = from.y();
    camToWorld(3, 2) = from.z();

    return camToWorld.transpose();
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene, CameraIO* in_camera)
{
    //framebuffer for output
    #define CHANNEL_NUM 3

    //framebuffer for the stb image
    uint8_t* stb_framebuffer = new uint8_t[scene.width * scene.height * CHANNEL_NUM];

    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    //Vector3f eye_pos(-1, 5, 10);
    int m = 0;
    int index = 0;
    int ix = 0;//for image buffer

    auto position = Eigen::Vector3f(in_camera->position);
    auto viewDirection = Eigen::Vector3f(in_camera->viewDirection);
    auto orthoUp = Eigen::Vector3f(in_camera->orthoUp);
    auto camToWorld = lookAt(position, viewDirection, orthoUp);

    /*auto position = Vec3f(in_camera->position);
    auto viewDirection = Vec3f(in_camera->viewDirection);
    auto orthoUp = Vec3f(in_camera->orthoUp);
    camera cam(position, viewDirection, orthoUp, 90, imageAspectRatio);*/



    for (int j = scene.height - 1; j >= 0; j--)
    {
        for (int i = 0; i < scene.width; i++)
        {
            
            
            int xsamples = 2;
            int ysamples = 2;
            Vec3f pc(0.0f, 0.0f, 0.0f);
            for (int m = 0; m < xsamples; ++m)
            {
                for (int n = 0; n < ysamples; ++n)
                {
                    float mdel = random_float(m, m + 1) / 2.0f;
                    float ndel = random_float(n, n + 1) / 2.0f;
                    float x = (2 * (i + mdel) / (float)scene.width - 1) * imageAspectRatio * scale;
                    float y = (1 - 2 * (j + ndel) / (float)scene.height) * scale;

                    Eigen::Vector4f orig(0.0f, 0.0f, 0.0f, 1.0f);
                    Eigen::Vector4f dir = Eigen::Vector4f(x, y, -1.0f, 0.0f).normalized();
                    Eigen::Vector4f rayWorldOrig4f = camToWorld * orig;
                    Eigen::Vector4f rayWorldDir4f = camToWorld * dir;

                    auto rayWorldOrig = Vector3f(rayWorldOrig4f.x(), rayWorldOrig4f.y(), rayWorldOrig4f.z());
                    auto rayWorldDir = Vector3f(rayWorldDir4f.x(), rayWorldDir4f.y(), rayWorldDir4f.z());

                    pc += scene.castRay(Ray(rayWorldOrig, rayWorldDir), 0);
                }
            }

            pc = pc / (xsamples * ysamples);
            Vec3f col = pc;
            
            
            int ired = int(255.99 * col.x);
            int igreen = int(255.99 * col.y);
            int iblue = int(255.99 * col.z);

            stb_framebuffer[ix++] = ired;
            stb_framebuffer[ix++] = igreen;
            stb_framebuffer[ix++] = iblue;
            
            
            /*float s = float(i) / float(scene.height);
            float t = float(j) / float(scene.width);

            float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
            //Vec3f dir = (Vec3f(x, y, -1)).normalized();

            Eigen::Vector4f orig(0.0f, 0.0f, 0.0f, 1.0f);
            Eigen::Vector4f dir = Eigen::Vector4f(x, y, -1.0f, 0.0f).normalized();
            Eigen::Vector4f rayWorldOrig4f = camToWorld * orig;
            Eigen::Vector4f rayWorldDir4f = camToWorld * dir;

            auto rayWorldOrig = Vector3f(rayWorldOrig4f.x(), rayWorldOrig4f.y(), rayWorldOrig4f.z());
            auto rayWorldDir = Vector3f(rayWorldDir4f.x(), rayWorldDir4f.y(), rayWorldDir4f.z());

            //Eigen::Vector3f orig(0);
            //Eigen::Vector3f dir = Eigen::Vector3f(x, y, -1.0f).normalized();

            //vec3 dir(lower_left_corner + s * horizontal + t * vertical);
            //dir.normalize();
            //ray r = cam.get_ray(s, t);
            auto col = scene.castRay(Ray(rayWorldOrig, rayWorldDir), 0);
            //col = castRay(orig, dir, objects, lights, 0);
            //col = vec3(0);

            int ired = int(255.99 * col.x);
            int igreen = int(255.99 * col.y);
            int iblue = int(255.99 * col.z);

            stb_framebuffer[ix++] = ired;
            stb_framebuffer[ix++] = igreen;
            stb_framebuffer[ix++] = iblue;*/
        }
        //UpdateProgress(j / (float)scene.height);
    }
    //UpdateProgress(1.f);

    /*for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x;
            float y;

            // generate primary ray direction
            float x_ndc = (i + 0.5) / scene.width;
            float y_ndc = (j + 0.5) / scene.height;

            //float x_screen = 2 * x_ndc - 1;
            //float y_screen = 1- 2 * y_ndc;

            float x_camera = (2 * x_ndc - 1) * imageAspectRatio * scale;
            float y_camera = (1 - 2 * y_ndc) * scale;
            // TODO: Find the x and y positions of the current pixel to get the
            // direction
            //  vector that passes through it.
            // Also, don't forget to multiply both of them with the variable
            // *scale*, and x (horizontal) variable with the *imageAspectRatio*

            // Don't forget to normalize this direction!
            Vector3f dir = Vector3f(x_camera, y_camera, -1);
            normalize(dir);
            Vector3f col = scene.castRay(Ray(eye_pos, dir), 0);
            framebuffer[m++] = col;

            int ired = int(255.99 * col.x);
            int igreen = int(255.99 * col.y);
            int iblue = int(255.99 * col.z);

            stb_framebuffer[index++] = ired;
            stb_framebuffer[index++] = igreen;
            stb_framebuffer[index++] = iblue;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);*/

    /* save out the image */
    stbi_write_jpg("output\\scene5_4spp.jpg", scene.width, scene.height, 3, stb_framebuffer, 100);
    /* */

    /* cleanup */
    delete[] stb_framebuffer;
}
