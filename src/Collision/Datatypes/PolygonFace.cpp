#include "PolygonFace.hpp"

namespace Physik {
	PolygonFace::PolygonFace(const std::vector<int>& vertexIndices) : vertexIndices(vertexIndices)
	{}

	int PolygonFace::size() const
	{
		return vertexIndices.size();
	}

	int PolygonFace::operator[](int index) const
	{
		return vertexIndices[index];
	}
}