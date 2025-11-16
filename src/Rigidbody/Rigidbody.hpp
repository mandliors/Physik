#pragma once

#include "Colliders/Collider.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>

struct Rigidbody
{
	Rigidbody();

	void CollideWith(Rigidbody& other);
	void ClearForces();
	void ApplyForces();

public:
	// constant properties
	double mass;
	glm::vec3 centerOfMass;
	glm::mat3 thetaBody;
	glm::mat3 thetaBodyInv;

	// state variables
	glm::vec3 position;
	glm::quat orientation;
	glm::vec3 linearMomentum;
	glm::vec3 angularMomentum;

	// derived quantities
	/*glm::mat3 thetaInv;
	glm::mat3 rotationMat;
	glm::vec3 velocity;
	glm::vec3 angularVelocity;*/

	// computed quantities
	glm::vec3 force;
	glm::vec3 torque;

	std::vector<std::reference_wrapper<Collider>> colliders;
};