#include "Rigidbody/Rigidbody.hpp"
#include "Collision/CollisionInfo.hpp"

Rigidbody::Rigidbody()
	: mass(1.0),
	  centerOfMass(3),
	  thetaBody(3, 3),
	  thetaBodyInv(3, 3),

	  position(3),
	  orientation(1.0, 0.0, 0.0, 0.0),
	  linearMomentum(3),
	  angularMomentum(3),

	  thetaInv(3, 3),
	  rotationMat(3, 3),
	  velocity(3),
	  angularVelocity(3),

	  force(3),
	  torque(3)
{
}

void Rigidbody::CollideWith(Rigidbody &other)
{
	std::vector<CollisionInfo> collisions;
	for (Collider &colliderA : colliders)
	{
		for (Collider &colliderB : other.colliders)
		{
			CollisionInfo info = colliderA.CollideWith(colliderB);
			if (info.isColliding)
				collisions.push_back(info);
		}
	}
}

void Rigidbody::CalculateDerivedQuantities()
{
	thetaInv = rotationMat * thetaBodyInv * rotationMat.transpose();
	rotationMat = orientation.toRotationMatrix();
	velocity = linearMomentum / static_cast<float>(mass);
	angularVelocity = thetaInv * angularMomentum;
}
void Rigidbody::CalculateStateDot(
	Eigen::Vector3d &positionDot, Eigen::Quaterniond &orientationDot,
	Eigen::Vector3d &linearMomentumDot, Eigen::Vector3d &angularMomentumDot) const
{
	positionDot = velocity;

	orientationDot = Eigen::Quaterniond(0.0, angularVelocity.x(), angularVelocity.y(), angularVelocity.z()) * orientation;
	orientationDot.coeffs() *= 0.5;

	linearMomentumDot = force;
	angularMomentumDot = torque;
}

void Rigidbody::ClearForces()
{
	force = Eigen::Vector3d(3);
	torque = Eigen::Vector3d(3);
}

void Rigidbody::ApplyForces()
{
}

Eigen::Vector3d Rigidbody::GetPointVelocity(const Eigen::Vector3d &point) const
{
	return velocity + angularVelocity.cross(point - position);
}