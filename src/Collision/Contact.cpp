#include "Contact.hpp"

Contact::Contact(Rigidbody &a, Rigidbody &b)
    : a(a), b(b), position(3), normal(3), edgeA(3), edgeB(3), isVertexFace(false)
{
}

bool Contact::IsColliding() const
{
    static constexpr float COLLISION_EPSILON = 0.01f;

    auto paVel = a.GetPointVelocity(position);
    auto pbVel = b.GetPointVelocity(position);
    auto relVel = pbVel - paVel;

    return relVel.dot(normal) < -COLLISION_EPSILON;
}

void Contact::DoCollisionResponse(float restitutionCoefficient)
{
    auto paVel = a.GetPointVelocity(position);
    auto pbVel = b.GetPointVelocity(position);
    auto relVel = pbVel - paVel;

    auto ra = position - a.position;
    auto rb = position - b.position;

    float velAlongNormal = normal.dot(relVel);
    float numerator = -(1.0f + restitutionCoefficient) * velAlongNormal;

    float term1 = 1.0f / a.mass;
    float term2 = 1.0f / b.mass;
    float term3 = normal.dot((a.thetaInv * (ra.cross(normal))).cross(ra));
    float term4 = normal.dot((b.thetaInv * (rb.cross(normal))).cross(rb));

    float j = numerator / (term1 + term2 + term3 + term4);
    auto impulse = j * normal;

    a.linearMomentum += impulse;
    b.linearMomentum -= impulse;
    a.angularMomentum += ra.cross(impulse);
    b.angularMomentum -= rb.cross(impulse);

    a.velocity = a.linearMomentum / static_cast<float>(a.mass);
    a.angularVelocity = a.thetaInv * a.angularMomentum;
}

Eigen::Vector3d Contact::ComputeNDot() const
{
    if (isVertexFace)
        return b.angularVelocity.cross(normal);

    auto eadot = a.angularVelocity.cross(edgeA);
    auto ebdot = b.angularVelocity.cross(edgeB);

    auto n = edgeA.cross(edgeB);
    double nLength = n.norm();
    n /= nLength;

    auto z = eadot.cross(edgeB) + edgeA.cross(ebdot);

    return (z - ((z.dot(n)) * n)) / nLength;
}