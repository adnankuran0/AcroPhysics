#include "Narrowphase.h"

using namespace Acro::Physics;
using namespace Acro::Math;
using Pair = std::pair<Acro::Physics::ShapeInstanceHandle, Acro::Physics::ShapeInstanceHandle>;


std::vector<ContactManifold> Narrowphase::Compute(
    Acro::Physics::ShapeManager& sm,
    Acro::Physics::ShapeInstanceManager& sim, 
    const std::vector<Pair>& pairs)
{
    std::vector<ContactManifold> out;

    auto& data = sim.GetData();

    for (auto& pair : pairs)
    {
        auto instanceA = pair.first;
        auto instanceB = pair.second;

        size_t denseIndexA = sim.GetDenseIndex(instanceA);
        size_t denseIndexB = sim.GetDenseIndex(instanceB);

        if (denseIndexA >= data.shapes.size() || denseIndexB >= data.shapes.size()) continue;

        const auto& filterA = data.filters[denseIndexA];
        const auto& filterB = data.filters[denseIndexB];
        if (!CollisonLayer::ShouldCollide(filterA, filterB)) continue;

        ShapeHandle shapeA = data.shapes[denseIndexA];
        ShapeHandle shapeB = data.shapes[denseIndexB];

        ContactManifold manifold;
        manifold.a = instanceA;
        manifold.b = instanceB;

        auto shapeTypeA = sm.GetShapeType(shapeA);
        auto shapeTypeB = sm.GetShapeType(shapeB);

        CollisionPair collisionPair = { shapeA,shapeB,instanceA,instanceB };

        bool hit = false;

        if (shapeTypeA == ShapeType::Sphere && shapeTypeB == ShapeType::Sphere)
            hit = SphereSphere(collisionPair,sm,sim,manifold);
        else if (shapeTypeA == ShapeType::Sphere && shapeTypeB == ShapeType::Box)
            hit = SphereBox(collisionPair, sm, sim, manifold);
        else if (shapeTypeA == ShapeType::Box && shapeTypeB == ShapeType::Sphere)
            hit = SphereBox(collisionPair, sm, sim, manifold); // change normal dir
        else if (shapeTypeA == ShapeType::Box && shapeTypeB == ShapeType::Box)
            hit = BoxBox(collisionPair, sm, sim, manifold);

        if (hit)
            out.push_back(manifold);

    }

    return out;
}

bool Narrowphase::SphereSphere(
    const Acro::Physics::CollisionPair& pair,
    ShapeManager& sm, 
    ShapeInstanceManager& sim,
    ContactManifold& manifold)
{

    Vector3 centerA = sim.GetWorldTransform(pair.shapeInstanceA) * Vector3(0.0f);
    Vector3 centerB = sim.GetWorldTransform(pair.shapeInstanceB) * Vector3(0.0f);

    float radiusA = sm.GetRadius(pair.shapeA);
    float radiusB = sm.GetRadius(pair.shapeB);

    Vector3 ab = centerB - centerA;
    float distance = ab.Length();
    float radius = radiusA + radiusB;

    if (distance > radius) return false;

    Vector3 normal = (distance > std::numeric_limits<float>::min()) ? (ab * (1.0f / distance)) : Vector3(1.0f, 0.0f, 0.0f);
    float penetration = radius - distance;

    Vector3 point = centerA + normal * radiusA;

    manifold.count = 1;
    manifold.points[0] = { point ,normal, penetration };

    return true;
}

bool Narrowphase::SphereBox(
    const CollisionPair& pair,
    ShapeManager& sm,
    ShapeInstanceManager& sim,
    ContactManifold& manifold)
{
    // TODO: Find better solution
    ShapeInstanceHandle sphereInstanceHandle;
    ShapeHandle sphereShapeHandle;
    ShapeInstanceHandle boxInstanceHandle;
    ShapeHandle boxShapeHandle;

    if (sm.GetShapeType(pair.shapeA) == ShapeType::Sphere)
    {
        // sphere -> box
        sphereInstanceHandle = pair.shapeInstanceA;
        sphereShapeHandle = pair.shapeA;
        boxInstanceHandle = pair.shapeInstanceB;
        boxShapeHandle = pair.shapeB;
    }
    else
    {
        // box -> sphere
        sphereInstanceHandle = pair.shapeInstanceB;
        sphereShapeHandle = pair.shapeB;
        boxInstanceHandle = pair.shapeInstanceA;
        boxShapeHandle = pair.shapeA;
    }

    auto& data = sim.GetData();

    Vector3 sphereCenter = sim.GetWorldTransform(sphereInstanceHandle) * Vector3::ZERO();
    float radius = sm.GetRadius(sphereShapeHandle);

    uint32_t boxDenseIndex = sim.GetDenseIndex(boxInstanceHandle);
    const Matrix4& worldBox = sim.GetWorldTransform(boxInstanceHandle);
    Matrix4 inverseBox = worldBox.Inversed();
    Vector3 halfExtents = sm.GetExtent(boxShapeHandle);

    Vector3 localCenter = inverseBox * sphereCenter;

    Vector3 clampedLocalCenter = Vector3::Clamp(localCenter,halfExtents * -1.0f,halfExtents);
    Vector3 closestPoint = worldBox * clampedLocalCenter;

    // sphere-point vec
    Vector3 delta = sphereCenter - closestPoint;
    float distanceSq = delta.Length2();
    float radiusSq = radius * radius;

    if (distanceSq > radiusSq) return false;

    float distance = delta.Length();
    float penetration;
    Vector3 normal;
    if (distance > std::numeric_limits<float>::min())
    {
        normal = delta * (1.0f / distance);
        penetration = radius - distance;
    }
    else
    {
        // sphere center inside box
        // BUGFIX: this creates multiple contact points or wrong normals
        normal = Vector3(1.0f, 0.0f, 0.0f);
        penetration = radius - distance;

    }
     
    Vector3 contactOnSphere = sphereCenter - normal * radius;
    Vector3 contactPoint = (contactOnSphere + closestPoint) * 0.5f;

    manifold.count = 1;
    manifold.points[0] = { contactPoint , normal , penetration };

    return true;
}

bool Narrowphase::BoxBox(
    const CollisionPair& pair,
    ShapeManager& sm,
    ShapeInstanceManager& sim, 
    ContactManifold& manifold)
{
    return false;
}
