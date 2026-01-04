#include "Scene.h"

using namespace Acro::Math;

void Scene::Init(Acro::World& world)
{
    Acro::BodyDescription groundDesc;
    groundDesc.position = Vector3(0.0f, -2.0f, 0.0f);
    groundDesc.mass = 0.0f;

    groundBody = world.CreateBody(groundDesc);
    Acro::BoxShape groundShape = world.CreateBoxShape(Vector3(10.0f, 0.1f, 10.0f));
    groundBody.AttachShape(groundShape);

    Acro::BoxShape cubeShape = world.CreateBoxShape(Vector3(0.5f, 0.5f, 0.5f));
    Acro::SphereShape sphereShape = world.CreateSphereShape(0.5f);

    const int rows = 6;
    const int cols = 6;
    const float spacing = 2.0f;

    for (int x = 0; x < cols; x++)
    {
        for (int z = 0; z < rows; z++)
        {
            float posX = -6.0f + x * spacing;
            float posZ = -4.0f + z * spacing;
            float height = 1.0f + (x + z) * 0.6f;

            Acro::BodyDescription d;
            d.position = Vector3(posX, height, posZ);
            d.mass = 1.0f;

            Acro::Rigidbody body = world.CreateBody(d);

            //if ((x + z) % 2 == 0)
                //body.AttachShape(cubeShape);
            //else
            body.AttachShape(sphereShape);

            bodies.push_back(body);

            initialStates.push_back({ d.position, d.orientation });
        }
    }
}

void Scene::Reset()
{
    requestReset = false;
    

    for (size_t i = 0; i < bodies.size(); i++)
    {
        bodies[i].SetPosition(initialStates[i].position);
        bodies[i].SetOrientation(initialStates[i].orientation);
        bodies[i].SetLinearVelocity(Vector3(0.0f, 0.0f, 0.0f));
        bodies[i].SetAngularVelocity(Vector3(0.0f, 0.0f, 0.0f));
    }

    groundBody.SetOrientation(Quaternion(Vector3(1.0f, 0.0f, 0.0f), 0.0f));
}

void Scene::Update(double glfwTime)
{
    groundBody.SetOrientation(Quaternion(Vector3(1.0f, 0.0f, 0.0f), glfwTime * 0.5f));
}