#pragma once

#include "Rigidbody/Rigidbody.hpp"

#include <Eigen/Dense>

namespace Physik
{
    struct Contact
    {
        enum class State
        {
            Separating = 0,
            Colliding,
            Resting
        };
        enum class Type
        {
            VertexFace = 0,
            EdgeEdge
        };

    public:
        Contact(Rigidbody &a, Rigidbody &b);

        State GetState() const;
        void DoCollisionResponse(float restitutionCoefficient);
        Eigen::Vector3d ComputeNDot() const;

        Rigidbody &a;
        Rigidbody &b;

        Eigen::Vector3d position;
        Eigen::Vector3d normal;
        Eigen::Vector3d edgeA;
        Eigen::Vector3d edgeB;

        Type type;
    };
}