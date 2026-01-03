#include "Solver.h"
#include "Math/Vector3.h"
#include "Math/Constants.h"

using namespace Acro::Math;
using namespace Acro::Physics;

void Solver::Solve(
    const std::vector<ContactManifold>& manifolds,
    ShapeInstanceManager& sim,
    BodyManager& bm,
    float deltaTime) noexcept
{
    for (int i = 0; i < m_Iterations; ++i)
        SolveVelocityConstraints(manifolds, sim, bm);

    SolvePositionConstraints(manifolds,sim,bm);
}


float Solver::ComputeNormalImpulse(
    float velocityNormal,
    float invMassSum,
    float restitution) noexcept
{
    float j = -(1.0f + restitution) * velocityNormal;
    j /= invMassSum;
    return j;
}

void Solver::ApplyImpulse(
    const BodyHandle& a,
    const BodyHandle& b,
    float invMassA,
    float invMassB,
    const Vector3& impulse,
    BodyManager& bm) noexcept
{
    bm.SetLinearVelocity(a, bm.GetLinearVelocity(a) - impulse * invMassA);
    bm.SetLinearVelocity(b, bm.GetLinearVelocity(b) + impulse * invMassB);
}

void Solver::SolveFriction(
    const Vector3& rv,
    const Vector3& normal,
    float normalImpulse,
    float invMassSum,
    const BodyHandle& a,
    const BodyHandle& b,
    BodyManager& bm) noexcept
{
    Vector3 tangentVel = rv - normal * rv.Dot(normal);
    float lenSq = tangentVel.Length2();
    if (lenSq < EPSILON) return;

    Vector3 tangent = tangentVel / std::sqrt(lenSq);

    float jt = -rv.Dot(tangent) / invMassSum;
    float mu = 0.5f;
    jt = std::clamp(jt, -normalImpulse * mu, normalImpulse * mu);

    ApplyImpulse(a, b, bm.GetInverseMass(a), bm.GetInverseMass(b), tangent * jt, bm);
}

void Solver::SolveContactPoint(
    const ContactManifold& manifold,
    const ContactPoint& point,
    ShapeInstanceManager& sim, 
    BodyManager& bm) noexcept
{
    auto bodyA = sim.GetData().bodies[sim.GetDenseIndex(manifold.a)];
    auto bodyB = sim.GetData().bodies[sim.GetDenseIndex(manifold.b)];

    float invMassA = bm.GetInverseMass(bodyA);
    float invMassB = bm.GetInverseMass(bodyB);
    float invMassSum = invMassA + invMassB;
    // return if both are static
    if (invMassSum == 0.0f) return;

    Vector3 vA = bm.GetLinearVelocity(bodyA);
    Vector3 vB = bm.GetLinearVelocity(bodyB);
    Vector3 rv = vB - vA;

    float vn = rv.Dot(point.normal);
    if (vn > 0.0f) return;

    float restitution = 0.5f;
    float restitutionThreshold = 0.5f;
    if (std::abs(vn) < restitutionThreshold) restitution = 0.0f;

    float j = ComputeNormalImpulse(vn, invMassSum, restitution);
    ApplyImpulse(bodyA, bodyB, invMassA, invMassB, point.normal * j,  bm);
    Vector3 newRv = bm.GetLinearVelocity(bodyB) - bm.GetLinearVelocity(bodyA);
    SolveFriction(newRv, point.normal, j, invMassSum, bodyA, bodyB, bm);
}

void Solver::SolveVelocityConstraints(
    const std::vector<ContactManifold>& manifolds, 
    ShapeInstanceManager& sim, 
    BodyManager& bm) noexcept
{
    for (const auto& manifold : manifolds)
        for (const auto& point : manifold.points)
            SolveContactPoint(manifold, point, sim, bm);
}

void Solver::SolvePositionConstraints(
    const std::vector<ContactManifold>& manifolds,
    ShapeInstanceManager& sim,
    BodyManager& bm) noexcept
{
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
