#include "Contact.hpp"

Contact::Contact(Rigidbody &a, Rigidbody &b)
    : a(a), b(b), position(3), normal(3), edgeA(3), edgeB(3), isVertexFace(false)
{
}

bool Contact::IsColliding() const
{
    static constexpr double COLLISION_EPSILON = 0.01;

    auto paVel = a.GetPointVelocity(position);
    auto pbVel = b.GetPointVelocity(position);
    auto relVel = pbVel - paVel;

    return relVel.dot(normal) < -COLLISION_EPSILON;
}

bool Contact::IsResting() const
{
#define COLLISION_EPSILON 0.01

    auto paVel = a.GetPointVelocity(position);
    auto pbVel = b.GetPointVelocity(position);
    auto relVel = pbVel - paVel;

    return abs(relVel.dot(normal)) <= COLLISION_EPSILON;
#undef COLLISION_EPSILON
}

void Contact::DoCollisionResponse(float restitutionCoefficient)
{
    auto paVel = a.GetPointVelocity(position);
    auto pbVel = b.GetPointVelocity(position);
    auto relVel = pbVel - paVel;

    auto ra = position - a.state.position;
    auto rb = position - b.state.position;

    double velAlongNormal = normal.dot(relVel);
    double numerator = -(1.0 + restitutionCoefficient) * velAlongNormal;

    double term1 = 1.0 / a.constants.mass;
    double term2 = 1.0 / b.constants.mass;
    double term3 = normal.dot((a.derived.thetaInv * (ra.cross(normal))).cross(ra));
    double term4 = normal.dot((b.derived.thetaInv * (rb.cross(normal))).cross(rb));

    double j = numerator / (term1 + term2 + term3 + term4);
    auto impulse = j * normal;

    a.state.linearMomentum += impulse;
    b.state.linearMomentum -= impulse;
    a.state.angularMomentum += ra.cross(impulse);
    b.state.angularMomentum -= rb.cross(impulse);

    // TODO: recalculating velocty and angularvelocity might be enough
    a.CalculateDerivedQuantities();
    b.CalculateDerivedQuantities();
}

Eigen::Vector3d Contact::ComputeNDot() const
{
    if (isVertexFace)
        return b.derived.angularVelocity.cross(normal);

    auto eadot = a.derived.angularVelocity.cross(edgeA);
    auto ebdot = b.derived.angularVelocity.cross(edgeB);

    auto n = edgeA.cross(edgeB);
    double nLength = n.norm();
    n /= nLength;

    auto z = eadot.cross(edgeB) + edgeA.cross(ebdot);

    return (z - ((z.dot(n)) * n)) / nLength;
}