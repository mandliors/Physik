#pragma once

#include "Rigidbody/Rigidbody.hpp"

#include <Eigen/Dense>

struct Contact
{
    Contact(Rigidbody &a, Rigidbody &b);

    bool IsColliding() const;
    void DoCollisionResponse(float restitutionCoefficient);
    Eigen::Vector3d ComputeNDot() const;

    Rigidbody &a;
    Rigidbody &b;

    Eigen::Vector3d position;
    Eigen::Vector3d normal;
    Eigen::Vector3d edgeA;
    Eigen::Vector3d edgeB;

    bool isVertexFace;
};