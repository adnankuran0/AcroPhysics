#include "Narrowphase.h"

#include <algorithm>

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

    Vector3 delta = centerB - centerA;
    float distanceSq = delta.Length2();
    float radius = radiusA + radiusB;
    float radiusSq = radius * radius;

    if (distanceSq > radiusSq) return false;

    float distance = delta.Length();

    Vector3 normal = (distance > std::numeric_limits<float>::epsilon()) ? (delta * (1.0f / distance)) : Vector3(1.0f, 0.0f, 0.0f);
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

    Vector3 clampedLocalCenter = Vector3::Clamp(localCenter,-halfExtents,halfExtents);

    bool isInside = 
        ((localCenter.x >= -halfExtents.x && localCenter.x <= halfExtents.x) &&
        (localCenter.y >= -halfExtents.y && localCenter.y <= halfExtents.y) &&
        (localCenter.z >= -halfExtents.z && localCenter.z <= halfExtents.z));

    Vector3 contactPoint;
    Vector3 normal;
    float penetration;


    if (isInside)
    {
        // find closest face
        float distX = std::min(halfExtents.x - localCenter.x, localCenter.x + halfExtents.x);
        float distY = std::min(halfExtents.y - localCenter.y, localCenter.y + halfExtents.y);
        float distZ = std::min(halfExtents.z - localCenter.z, localCenter.z + halfExtents.z);

        float minDist;
        Vector3 localNormal;

        if (distX < distY && distX < distZ)
        {
            localNormal = (localCenter.x > 0) ? Vector3(1.0f,0.0f,0.0f) : Vector3(-1.0f, 0.0f, 0.0f);
            clampedLocalCenter.x = (localCenter.x > 0) ? halfExtents.x : -halfExtents.x;
            minDist = distX;
        }
        else if (distY < distZ)
        {
            localNormal = (localCenter.y > 0) ? Vector3(0.0f, 1.0f, 0.0f) : Vector3(0.0f, -1.0f, 0.0f);
            clampedLocalCenter.y = (localCenter.y > 0) ? halfExtents.y : -halfExtents.y;
            minDist = distY;
        }
        else // disZ min
        {
            localNormal = (localCenter.z > 0) ? Vector3(0.0f, 0.0f, 1.0f) : Vector3(0.0f, 0.0f, -1.0f);
            clampedLocalCenter.z = (localCenter.z > 0) ? halfExtents.z : -halfExtents.z;
            minDist = distZ;
        }

        // to world space
        Vector3 localNormalEnd = Vector3::ZERO() + localNormal;

        normal = (worldBox * localNormalEnd - worldBox * Vector3::ZERO()).Normalized();
        penetration = radius + minDist;
        contactPoint = worldBox * clampedLocalCenter; // closest face point

    }
    else
    {
        Vector3 closestPoint = worldBox * clampedLocalCenter;
        Vector3 delta = sphereCenter - closestPoint;

        float distanceSq = delta.Length2();
        float radiusSq = radius * radius;

        if (distanceSq > radiusSq) return false;

        float distance = delta.Length();
        if (distance > std::numeric_limits<float>::epsilon())
        {
            normal = delta * (1.0f / distance);
            
        }
        else
        {
            normal = Vector3(1.0f, 0.0f, 0.0f);
            
        }
        penetration = radius - distance;
        contactPoint = closestPoint;
    }
   

    manifold.count = 1;
    manifold.points[0] = { contactPoint, normal , penetration };

    return true;
}

bool Narrowphase::BoxBox(
    const CollisionPair& pair,
    ShapeManager& sm,
    ShapeInstanceManager& sim, 
    ContactManifold& manifold)
{
    auto& data = sim.GetData();

    ShapeInstanceHandle instanceA = pair.shapeInstanceA;
    ShapeInstanceHandle instanceB = pair.shapeInstanceB;
    ShapeHandle shapeA = pair.shapeA;
    ShapeHandle shapeB = pair.shapeB;

    const Matrix4& worldA = sim.GetWorldTransform(instanceA);
    const Matrix4& worldB = sim.GetWorldTransform(instanceB);

    Vector3 extentA = sm.GetExtent(pair.shapeA);
    Vector3 extentB = sm.GetExtent(pair.shapeB);

    Vector3 centerA = worldA * Vector3::ZERO();
    Vector3 centerB = worldB * Vector3::ZERO();

    Vector3 axesA[3] = {
        (worldA * Vector3(1.0f,0.0f,0.0f) - centerA).Normalize(),
        (worldA * Vector3(0.0f,1.0f,0.0f) - centerA).Normalize(),
        (worldA * Vector3(0.0f,0.0f,1.0f) - centerA).Normalize(),
    };
    Vector3 axesB[3] = {
        (worldB * Vector3(1.0f,0.0f,0.0f) - centerA).Normalize(),
        (worldB * Vector3(0.0f,1.0f,0.0f) - centerA).Normalize(),
        (worldB * Vector3(0.0f,0.0f,1.0f) - centerA).Normalize(),
    };

    Vector3 delta = centerB - centerA;

    /* for 3D SAT
        3 axes for A
        3 axes for B
        9 cross product axes
        total 15 axes
    */

    float minPenetration = std::numeric_limits<float>::max();
    Vector3 minAxis;
    int minAxisType = -1; // 0: A Face , 1: B Face , 2: edge-edge

    auto TestAxis = [&](const Acro::Math::Vector3& axis, int axisType) {

        if (axis.Length2() < std::numeric_limits<float>::epsilon()) return true; // degen axis

        Vector3 normalizedAxis = axis.Normalized();


        float projA = 
            extentA.x * std::abs(axesA[0].Dot(normalizedAxis)) +
            extentA.y * std::abs(axesA[1].Dot(normalizedAxis)) + 
            extentA.z * std::abs(axesA[2].Dot(normalizedAxis));

        float projB =
            extentB.x * std::abs(axesA[0].Dot(normalizedAxis)) +
            extentB.y * std::abs(axesA[1].Dot(normalizedAxis)) +
            extentB.z * std::abs(axesA[2].Dot(normalizedAxis));

        float distance = std::abs(delta.Dot(normalizedAxis));

        // check overlap
        float penetration = projA + projB - distance;

        if (penetration < 0.0f) return false; // found separating axis, no collision

        // keep track of min penetration
        if (penetration < minPenetration)
        {
            minPenetration = penetration;
            minAxis = normalizedAxis;
            minAxisType = axisType;

            // make normal point to A
            if (delta.Dot(normalizedAxis) < 0.0f)
                minAxis = -minAxis;
        }

        return true; 

        };

    // test axes

    // face normals of A
    for (int i = 0; i < 3; i++)
    {
        if (!TestAxis(axesA[i], 0)) return false;
    }

    // face normals of B
    for (int i = 0; i < 3; i++)
    {
        if (!TestAxis(axesB[i], 1)) return false;
    }

    // edge-edge cross products 
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Vector3 axis = axesA[i].Cross(axesB[j]);
            if (!TestAxis(axesB[i], 2)) return false;
        }
    }

    // found collision

    manifold.count = 0;

    // find contact points

    if (minAxisType == 0 || minAxisType == 1) // face face contact
    {
        // reference faces and incident faces
        Matrix4 refWorld, incWorld;
        Vector3 refExtent, incExtent;
        Vector3 refNormal;
        int refAxisIndex;


        if (minAxisType == 0)
        {
            refWorld = worldA;
            incWorld = worldB;
            refExtent = extentA;
            incExtent = extentB;

            // find best aligned axis
            refAxisIndex = 0;
            float maxDot = std::abs(minAxis.Dot(axesA[0]));

            for (int i = 0; i < 3; i++)
            {
                float d = std::abs(minAxis.Dot(axesA[i]));
                if (d > maxDot)
                {
                    maxDot = d;
                    refAxisIndex = i;
                }
            }
            refNormal = axesA[refAxisIndex];

            if (refNormal.Dot(minAxis) < 0) refNormal = -refNormal;
        }
        else 
        {
            refWorld = worldB;
            incWorld = worldA;
            refExtent = extentB;
            incExtent = extentA;

            // find best aligned axis
            refAxisIndex = 0;
            float maxDot = std::abs(minAxis.Dot(axesB[0]));

            for (int i = 0; i < 3; i++)
            {
                float d = std::abs(minAxis.Dot(axesB[i]));
                if (d > maxDot)
                {
                    maxDot = d;
                    refAxisIndex = i;
                }
            }
            refNormal = axesB[refAxisIndex];

            if (refNormal.Dot(minAxis) < 0) refNormal = -refNormal;
        }

        //TODO: for now just 1 contact point with using center point
        Vector3 refCenter = refWorld * Vector3::ZERO();
        Vector3 incCenter = incWorld * Vector3::ZERO();

        Vector3 refExtentVec;
        refExtentVec[refAxisIndex] = refExtent[refAxisIndex];
        Vector3 faceCenter = refWorld * refExtentVec;

        // contact point : project incident box center to reference face
        float dist = (incCenter - faceCenter).Dot(refNormal);
        Vector3 contactPoint = incCenter - refNormal * dist;

        manifold.count = 1;
        manifold.points[0] = { contactPoint,minAxis,minPenetration };
    }
    else // edge edge contact
    {
        // TODO: find real contact points
        Vector3 contactPoint = centerA + (minAxis * (minPenetration) * 0.5f);
        manifold.count = 1;
        manifold.points[0] = { contactPoint,minAxis,minPenetration };
    }

    return true;
}
