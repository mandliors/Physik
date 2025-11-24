#pragma once

#include <Eigen/Dense>
#include <vector>

namespace Physik {
	class PolygonFace {
	private:
		std::vector<Eigen::Vector3d> vertices;		//counter-clockwise

	public:
		PolygonFace(const std::vector<Eigen::Vector3d>& vertices);

		int size() const;

		Eigen::Vector3d& operator[](int index);
	};
}