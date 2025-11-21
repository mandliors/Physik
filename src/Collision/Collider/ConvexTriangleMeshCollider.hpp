#pragma once

#include "BaseCollider.hpp"

#include <vector>
#include <Eigen/Dense>

class ConvexTriangleMeshCollider : BaseCollider {
public:
	const std::vector<Eigen::Vector3d> vertices;
	const std::vector<long> triangles;	//a flattened array of integer triplets

	ConvexTriangleMeshCollider(const std::vector<Eigen::Vector3d>& vertices, const std::vector<long>& triangles);

	void CalculateBoundingSphere();

	//returns the separation distance
	//the most segregating axis is written in outAxis, even if the colliders overlap
	//if absolutely no axis is found, the return value is DBL_MIN
	double FindSeparatingAxis(const ConvexTriangleMeshCollider& other, BaseCollider::SeparatingAxis& outAxis);
};
