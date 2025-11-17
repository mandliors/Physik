#include "Rigidbody/Rigidbody.hpp"
#include "Colliders/CollisionInfo.hpp"

Rigidbody::Rigidbody()
	: mass(1.0),
	  centerOfMass(0.0f),
	  thetaBody(1.0f),
	  thetaBodyInv(1.0f),

	  position(0.0f),
	  orientation(1.0f, 0.0f, 0.0f, 0.0f),
	  linearMomentum(0.0f),
	  angularMomentum(0.0f),

	  thetaInv(1.0f),
	  rotationMat(1.0f),
	  velocity(0.0f),
	  angularVelocity(0.0f),

	  force(0.0f),
	  torque(0.0f)
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
	thetaInv = rotationMat * thetaBodyInv * glm::transpose(rotationMat);
	rotationMat = glm::mat3_cast(orientation);
	velocity = linearMomentum / static_cast<float>(mass);
	angularVelocity = thetaInv * angularMomentum;
}
void Rigidbody::CalculateStatePrime(
	glm::vec3 &positionPrime, glm::quat &orientationPrime,
	glm::vec3 &linearMomentumPrime, glm::vec3 &angularMomentumPrime) const
{
	positionPrime = velocity;
	orientationPrime = 0.5f * glm::quat(0.0f, angularVelocity) * orientation;
	linearMomentumPrime = force;
	angularMomentumPrime = torque;
}

void Rigidbody::ClearForces()
{
	force = glm::vec3(0.0f);
	torque = glm::vec3(0.0f);
}

void Rigidbody::ApplyForces()
{
}