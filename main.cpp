#include "Renderer.hpp"
//#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>
#include "scene_io.h"
#include "Sphere.hpp"
#include "Timer.h"

SceneIO* scene = NULL;

void loadScene(std::string name) {
    /* load the scene into the SceneIO data structure using given parsing code */
    scene = readScene(name.c_str());

    /* hint: use the Visual Studio debugger ("watch" feature) to probe the
       scene data structure and learn more about it for each of the given scenes */


       /* write any code to transfer from the scene data structure to your own here */
       /* */

    return;
}

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
	Timer total_timer, bvh_timer, render_timer;
	total_timer.startTimer();

    loadScene("../Scenes/test5.ascii");

    Scene scene_main(1500, 1500);
	
	bvh_timer.startTimer();
	LightIO* currLight = scene->lights;
	ObjIO* currObj = scene->objects;

	while (currLight != NULL)
	{

		std::unique_ptr<Light> l = std::make_unique<Light>(
			Vec3f(currLight->position),
			Vec3f(currLight->direction),
			Vec3f(currLight->color),
			currLight->type);

		scene_main.Add(std::move(l));
		currLight = currLight->next;
	}


	
	int i = 0;
	while (currObj != NULL)
	{
		if (currObj->type == SPHERE_OBJ)
		{
			Material m;
			m.diffuseColor = Vec3f(currObj->material->diffColor);
			m.ka = currObj->material->ambColor[0];
			m.ks = currObj->material->specColor[0];
			m.ktran = currObj->material->ktran;
			m.specularExponent = currObj->material->shininess;

			SphereIO* currSphere = (SphereIO*)currObj->data;
			Sphere* mySphere = new Sphere((Vec3f)currSphere->origin, currSphere->radius, m);
			scene_main.Add(mySphere);
		}

		else if (currObj->type == POLYSET_OBJ)
		{

			std::vector<Vec3f> points;
			std::vector<Vec3f> normals;
			std::vector<uint32_t> indices;
			std::vector<Material> materials;
			std::vector<uint32_t> materialIds;

			std::vector<Vec2f> stVec;
			Vec2f stVals[6] = {
				Vec2f(0,0), Vec2f(0,1), Vec2f(1,0),
				Vec2f(1,1), Vec2f(0,0), Vec2f(1,0)
			};



			PolySetIO* currPoly = (PolySetIO*)currObj->data;
			if (currPoly->type == POLYSET_TRI_MESH)
			{
				std::vector<Vec3f> points;
				std::vector<Vec3f> normals;
				std::vector<uint32_t> indices;
				std::vector<Material> materials;
				std::vector<uint32_t> materialIds;

				std::vector<Vec2f> stVec;
				Vec2f stVals[6] = {
					Vec2f(0,0), Vec2f(0,1), Vec2f(1,0),
					Vec2f(1,1), Vec2f(0,0), Vec2f(1,0)
				};


				int id = 0;
				auto poly = currPoly->poly;
				for (int i = 0; i < currPoly->numPolys; ++i) {
					//auto poly = currPoly->poly++;

					for (int j = 0; j < poly->numVertices; ++j) {
						//auto v = poly->vert[id];
						auto vert = Vec3f(poly->vert[j].pos);
						auto norm = Vec3f(poly->vert[j].norm);
						auto st = Vec2f(poly->vert[j].s, poly->vert[j].t);
						auto mId = poly->vert[j].materialIndex;
						points.push_back(vert);
						normals.push_back(norm);
						materialIds.push_back(mId);

						int stId = id % 6;
						stVec.push_back(stVals[stId]);

						indices.push_back(id++);
					}


					poly++;
				}

				auto currMat = currObj->material;
				int matC = currObj->numMaterials;
				while (matC > 0)
				{
					Material m;
					m.diffuseColor = Vec3f(currMat->diffColor);
					m.ka = currMat->ambColor[0];
					m.ks = currMat->specColor[0];
					m.ktran = currMat->ktran;
					m.specularExponent = currMat->shininess;
					materials.push_back(m);

					currMat++;
					matC--;
				}


				MeshTriangle* mesh = new MeshTriangle(
					points, normals, indices,
					currPoly->numPolys,
					stVec, materials, materialIds,
					currPoly->normType, currPoly->materialBinding
				);
				scene_main.Add(mesh);
			}
		}

		currObj = currObj->next;
	}
	scene_main.buildBVH();
	bvh_timer.stopTimer();
	//end:external scene creator
    
	Renderer r;

    //auto start = std::chrono::system_clock::now();
	render_timer.startTimer();
    r.Render(scene_main, scene->camera);
	render_timer.stopTimer();
    //auto stop = std::chrono::system_clock::now();

    /*std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";*/
	total_timer.stopTimer();
	fprintf(stderr, "BVH time: %.5lf secs\n\n", bvh_timer.getTime());
	fprintf(stderr, "Render time: %.5lf secs\n\n", render_timer.getTime());
	fprintf(stderr, "Total time: %.5lf secs\n\n", total_timer.getTime());
    return 0;
}