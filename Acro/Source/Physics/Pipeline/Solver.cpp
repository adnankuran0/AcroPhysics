#include "Solver.h"
#include "Math/Vector3.h"

using namespace Acro::Math;

void Acro::Physics::Solver::Solve(
    const std::vector<Acro::Physics::ContactManifold>& manifolds,
    Acro::Physics::ShapeInstanceManager& sim,
    Acro::Physics::BodyManager& bm,
    float deltaTime) noexcept
{
    for (int i = 0; i < m_Iterations; i++)
    {
        for (const auto& manifold : manifolds)
        {
            for (const auto& point : manifold.points)
            {
                const auto& bodyHandleA = sim.GetData().bodies[sim.GetDenseIndex(manifold.a)];
                const auto& bodyHandleB = sim.GetData().bodies[sim.GetDenseIndex(manifold.b)];

                float invMassA = bm.GetInverseMass(bodyHandleA);
                float invMassB = bm.GetInverseMass(bodyHandleB);
                float invMassSum = invMassA + invMassB;

                // continue if both are static
                if (invMassSum == 0.0f) continue;

                Vector3 relativeVel = bm.GetLinearVelocity(bodyHandleB) - bm.GetLinearVelocity(bodyHandleA);
                float velocityNormal = relativeVel.Dot(point.normal);

                if (velocityNormal > 0.0f) continue;

                float restitution = 0.5f;

                float restitutionThreshold = 0.5f;
                if (std::abs(velocityNormal) < restitutionThreshold)
                    restitution = 0.0f;

                float j = -(1.0f + restitution) * velocityNormal;
                j /= invMassSum;

                Vector3 impulse = point.normal * j;

                bm.SetLinearVelocity(bodyHandleA, bm.GetLinearVelocity(bodyHandleA) - impulse * invMassA);
                bm.SetLinearVelocity(bodyHandleB, bm.GetLinearVelocity(bodyHandleB) + impulse * invMassB);
            }
        }
    }

    // Position correction 
    for (const auto& manifold : manifolds)
    {
        const auto& bodyHandleA = sim.GetData().bodies[sim.GetDenseIndex(manifold.a)];
        const auto& bodyHandleB = sim.GetData().bodies[sim.GetDenseIndex(manifold.b)];

        float invMassA = bm.GetInverseMass(bodyHandleA);
        float invMassB = bm.GetInverseMass(bodyHandleB);
        float invMassSum = invMassA + invMassB;

        if (invMassSum == 0.0f) continue;

        float percent = 0.2f;  
        float slop = 0.01f;    

        float correctionMag = std::max(manifold.points[0].penetration - slop, 0.0f) * percent / invMassSum;
        Vector3 correction = manifold.points[0].normal * correctionMag;

        Vector3 posA = bm.GetPosition(bodyHandleA);
        Vector3 posB = bm.GetPosition(bodyHandleB);

        posA -= correction * invMassA;
        posB += correction * invMassB;

        bm.SetPosition(bodyHandleA, posA);
        bm.SetPosition(bodyHandleB, posB);
    }
}
