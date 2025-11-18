#pragma once

#include "Rigidbody/Rigidbody.hpp"
#include "Collision/Contact.hpp"

#include <Eigen/Dense>

#include <vector>

struct Solver
{
	void Solve(const std::vector<std::reference_wrapper<Rigidbody>> &bodies) const;
	void FindAllCollisions(std::vector<Contact> &contacts) const;
	void ComputeContactForces(const std::vector<Contact> &restingContacts, float t) const;

	Eigen::VectorXd ComputeBVector(const std::vector<Contact> &contacts) const;

	Eigen::MatrixXd ComputeAMatrix(const std::vector<Contact> &contacts) const;
	double ComputeAAt(const Contact &ci, const Contact &cj) const;

	Eigen::VectorXd SolveQuadratic(const Eigen::MatrixXd &A, const Eigen::VectorXd &b) const;

	void OdeDiscontinuous() const;
};