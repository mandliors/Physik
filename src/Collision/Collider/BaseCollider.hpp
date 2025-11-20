#pragma once

#include <Eigen/Dense>

class BaseCollider {
public:
	class BoundingSphere {
	public:
		double radius;

		BoundingSphere();
	};

	class SeparatingPlane {
	public:
		std::reference_wrapper<const BaseCollider> a, b;
		Eigen::Vector3d point;
		Eigen::Vector3d normal;		//points from b to a

		SeparatingPlane(const BaseCollider& a, const BaseCollider& b, const Eigen::Vector3d& point, const Eigen::Vector3d& normal);
	};

	BoundingSphere bounds;
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

	BaseCollider();

	virtual void CalculateBoundingSphere() = 0;
};