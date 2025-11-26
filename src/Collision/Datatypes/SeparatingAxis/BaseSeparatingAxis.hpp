#pragma once

#include <Eigen/Dense>

namespace Physik {
	class BaseCollider;

	class BaseSeparatingAxis {
	public:
		const BaseCollider* a, * b;

		BaseSeparatingAxis();

		BaseSeparatingAxis(const BaseCollider* a, const BaseCollider* b);

		BaseSeparatingAxis& operator=(const BaseSeparatingAxis& other);

		virtual Eigen::Vector3d GetAxisDir() const = 0;	//points from a to b
		virtual Eigen::Vector3d GetReferencePlanePoint() const = 0;
		virtual ~BaseSeparatingAxis() = 0;
	};
}