#include "PolygonFace.hpp"

namespace Physik {
	PolygonFace::PolygonFace(const std::vector<Eigen::Vector3d>& vertices) : vertices(vertices)
	{}

	int PolygonFace::size() const
	{
		return vertices.size();
	}

	Eigen::Vector3d& PolygonFace::operator[](int index)
	{
		return vertices[index];
	}
}