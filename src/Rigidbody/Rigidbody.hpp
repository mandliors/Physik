#pragma once

#include "Collision/Collider.hpp"

#include <Eigen/Dense>

#include <vector>

struct Rigidbody
{
	Rigidbody();

	void CollideWith(Rigidbody &other);

	void CalculateDerivedQuantities();
	void CalculateStateDot(
		Eigen::Vector3d &positionDot, Eigen::Quaterniond &orientationDot,
		Eigen::Vector3d &linearMomentumDot, Eigen::Vector3d &angularMomentumDot) const;

	void ClearForces();
	void ApplyForces();

	Eigen::Vector3d GetPointVelocity(const Eigen::Vector3d &point) const;

public:
	// constant properties
	double mass;
	Eigen::Vector3d centerOfMass;
	Eigen::Matrix3d thetaBody;
	Eigen::Matrix3d thetaBodyInv;

	// state variables
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
	Eigen::Vector3d linearMomentum;
	Eigen::Vector3d angularMomentum;

	// derived quantities
	Eigen::Matrix3d thetaInv;
	Eigen::Matrix3d rotationMat;
	Eigen::Vector3d velocity;
	Eigen::Vector3d angularVelocity;

	// computed quantities
	Eigen::Vector3d force;
	Eigen::Vector3d torque;

	std::vector<std::reference_wrapper<Collider>> colliders;
};