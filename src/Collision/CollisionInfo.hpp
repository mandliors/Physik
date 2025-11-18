#pragma once

#include "Collider.hpp"

#include <Eigen/Dense>

struct CollisionInfo
{
	bool isColliding;
	Eigen::Vector3d contactPoint;
	Eigen::Vector3d collisionNormal;
	double penetrationDepth;

	const Collider &colliderA;
	const Collider &colliderB;
};