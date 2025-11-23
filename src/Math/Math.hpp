#pragma once

#include "Collision/Contact.hpp"

#include <Eigen/Dense>

#include <vector>

namespace Physik::Math
{
    void CalculateMassProperties(const Mesh &mesh, double density, Rigidbody::MassProperties &massProps);

    void SolveCollidingContacts(std::vector<Contact> &contacts);
    void SolveRestingContacts(const std::vector<Contact> &restingContacts, double t);

    Eigen::VectorXd ComputeBVector(const std::vector<Contact> &contacts);
    Eigen::MatrixXd ComputeAMatrix(const std::vector<Contact> &contacts);
    double ComputeAAt(const Contact &ci, const Contact &cj);

    Eigen::VectorXd SolveQuadratic(const Eigen::MatrixXd &A, const Eigen::VectorXd &b);
}