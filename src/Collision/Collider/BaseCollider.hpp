#pragma once

#include <Eigen/Dense>

namespace Physik {

	class BaseCollider {
	public:
		class BoundingSphere {
		public:
			double radius;

			BoundingSphere();
		};

		BoundingSphere bounds;
		Eigen::Vector3d position;
		Eigen::Quaterniond orientation;

		BaseCollider();

		virtual void CalculateBoundingSphere() = 0;
	};

}