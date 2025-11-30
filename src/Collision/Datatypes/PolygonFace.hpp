#pragma once

#include <Eigen/Dense>
#include <vector>

namespace Physik {
	class PolygonFace {
	private:
		std::vector<int> vertexIndices;		//counter-clockwise

	public:
		PolygonFace(const std::vector<int>& vertexIndices);

		int size() const;

		int operator[](int index) const;	//vertexIndices[index%size()]
	};
}