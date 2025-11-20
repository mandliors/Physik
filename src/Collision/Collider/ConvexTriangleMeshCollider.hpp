#pragma once

#include "BaseCollider.hpp"

#include <vector>
#include <Eigen/Dense>

class ConvexTriangleMeshCollider : BaseCollider {
public:
	const std::vector<Eigen::Vector3d>& vertices;
	const std::vector<long>& triangles;	//a flattened array of integer triplets

	ConvexTriangleMeshCollider(const std::vector<Eigen::Vector3d>& vertices, const std::vector<long>& triangles);

	void CalculateBoundingSphere();

	//if a plane is found, outPlane is overwritten and the return value is true
	//else the return value is false and the outPlane is touch starved
	//other is considered as collider b in the separating plane, that is the normal will point from other to this
	bool FindSeparatingPlane(const ConvexTriangleMeshCollider& other, BaseCollider::SeparatingPlane& outPlane);
};