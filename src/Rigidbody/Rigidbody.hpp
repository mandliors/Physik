#pragma once

#include "Mesh/Mesh.hpp"
#include "Solvers/OdeSolver.hpp"

#include <Eigen/Dense>

#include <vector>

namespace Physik
{
	struct Rigidbody
	{
		struct MassProperties
		{
			double inverseMass;
			Eigen::Vector3d centerOfMass;
			Eigen::Matrix3d inverseInertiaBody;
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
			Eigen::Matrix3d inverseInertia;
			Eigen::Matrix3d rotationMatrix;
			Eigen::Vector3d velocity;
			Eigen::Vector3d angularVelocity;
		};

	public:
		Rigidbody(const Mesh &mesh, double density);

		void CalculateDerivedQuantities();
		StateDot CalculateStateDot() const;

		void Step(const Solvers::OdeSolver &solver, double deltaTime);

		void ApplyForces();
		void ClearForces();

		Eigen::Vector3d GetPointVelocity(const Eigen::Vector3d &point) const;

	public:
		MassProperties constants;
		State state;
		DerivedQuantities derived;

		Eigen::Vector3d force;
		Eigen::Vector3d torque;

		Mesh mesh;
	};
}