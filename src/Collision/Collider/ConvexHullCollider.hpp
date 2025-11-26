#pragma once

#include "BaseCollider.hpp"
#include "../Datatypes/PolygonFace.hpp"

#include <vector>
#include <Eigen/Dense>

namespace Physik {

	class ConvexHullConvexHullSeparatingAxis;

	class ConvexHullCollider : public BaseCollider {
	public:
		const std::vector<Eigen::Vector3d> vertices;
		std::vector<long> triangles;		//a flattened array of integer triplets (mergeduplicates requires non-const field)
		const Eigen::Vector3d center;		//average of the vertices

		std::vector<PolygonFace> polygons;

	public:
		ConvexHullCollider(const std::vector<Eigen::Vector3d>& vertices, const std::vector<long>& triangles);

		void CalculateBoundingSphere();

		ConvexHullConvexHullSeparatingAxis CalculateSeparatingAxis(const ConvexHullCollider& other);

		//returns the separation distance
		//the most segregating axis is written in outAxis, even if the colliders overlap
		//if absolutely no axis is found, the return value is DBL_MIN
		double FindSeparatingAxis(const ConvexHullCollider& other, ConvexHullConvexHullSeparatingAxis& outAxis);

	private:
		//merges duplicate vertices
		void MergeDuplicates();

		//merges coplanar and adjacent triangles into convex polygons
		//assumes an already convex triangle mesh (otherwise concave polygons might occur)
		//call MergeDuplicates beforehand for optimal results
		void Polygonalize();
	};
}
