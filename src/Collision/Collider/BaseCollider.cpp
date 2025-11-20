#include "BaseCollider.hpp"

BaseCollider::BoundingSphere::BoundingSphere() :radius(0.0)
{}

BaseCollider::SeparatingPlane::SeparatingPlane(const BaseCollider& a, const BaseCollider& b, const Eigen::Vector3d& point, const Eigen::Vector3d& normal)
	:a(a), b(b), point(point), normal(normal.normalized())
{}

BaseCollider::BaseCollider()
{}