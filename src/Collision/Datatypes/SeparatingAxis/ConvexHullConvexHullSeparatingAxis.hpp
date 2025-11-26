#pragma once

#include "BaseSeparatingAxis.hpp"
#include "../PolygonFace.hpp"

namespace Physik {

	class ConvexHullCollider;

	class ConvexHullConvexHullSeparatingAxis : public BaseSeparatingAxis {
	public:
		bool isVertexFace;
		union {
			struct {
				int vertexIndex;		//vertex in b
				PolygonFace face;		//face in a
			} vfInfo;
			struct {
				int aEdgeIndices[2];
				int bEdgeIndices[2];
			} eeInfo;
		};

		ConvexHullConvexHullSeparatingAxis();

		ConvexHullConvexHullSeparatingAxis(const ConvexHullCollider* a, const ConvexHullCollider* b,
			const PolygonFace& aFace, int bVertexIndex);

		ConvexHullConvexHullSeparatingAxis(const ConvexHullCollider* a, const ConvexHullCollider* b,
			int ea0, int ea1, int eb0, int eb1);

		~ConvexHullConvexHullSeparatingAxis();

		Eigen::Vector3d GetAxisDir() const;
		Eigen::Vector3d GetReferencePlanePoint() const;
	};
}