#pragma once

#include <Eigen/Dense>

namespace Physik {
	class BaseCollider;

	class MeshContact {
	public:
		const BaseCollider* a, * b;
		Eigen::Vector3d point;
		Eigen::Vector3d normal;		//normalized, points from a to b

		MeshContact(const BaseCollider* a, const BaseCollider* b, const Eigen::Vector3d& point, const Eigen::Vector3d& normal);
	};
}