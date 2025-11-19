#pragma once

#include "Solvers/OdeSolver.hpp"

#include <Eigen/Dense>

#include <vector>

struct Rigidbody
{
	struct ConstantQuantities
	{
		double mass;
		Eigen::Vector3d centerOfMass;
		Eigen::Matrix3d thetaBody;
		Eigen::Matrix3d thetaBodyInv;
	};

	struct State
	{
		Eigen::Vector3d position;
		Eigen::Quaterniond orientation;
		Eigen::Vector3d linearMomentum;
		Eigen::Vector3d angularMomentum;
	};
	struct StateDot
	{
		Eigen::Vector3d positionDot;
		Eigen::Quaterniond orientationDot;
		Eigen::Vector3d linearMomentumDot;
		Eigen::Vector3d angularMomentumDot;
	};

	struct DerivedQuantities
	{
		Eigen::Matrix3d thetaInv;
		Eigen::Matrix3d rotationMat;
		Eigen::Vector3d velocity;
		Eigen::Vector3d angularVelocity;
	};

public:
	Rigidbody();

	void CalculateDerivedQuantities();
	StateDot CalculateStateDot() const;

	void Step(const OdeSolver &solver, double deltaTime);

	void ClearForces();
	void ApplyForces();

	Eigen::Vector3d GetPointVelocity(const Eigen::Vector3d &point) const;

public:
	ConstantQuantities constants;
	State state;
	DerivedQuantities derived;

	Eigen::Vector3d accumForce;
	Eigen::Vector3d accumTorque;

	Eigen::Vector3d force;
	Eigen::Vector3d torque;
};