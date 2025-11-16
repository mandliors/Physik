#pragma once

#include "Collider.hpp"

#include <glm/glm.hpp>

struct CollisionInfo
{
	bool isColliding;
	glm::vec3 contactPoint;
	glm::vec3 collisionNormal;
	float penetrationDepth;

	const Collider& colliderA;
	const Collider& colliderB;
};