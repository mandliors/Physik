#include "ConvexHullConvexHullSeparatingAxis.hpp"
#include "../../Collider/BaseCollider.hpp"
#include "../../Collider/ConvexHullCollider.hpp"
#include "../PolygonFace.hpp"

namespace Physik {

	ConvexHullConvexHullSeparatingAxis::ConvexHullConvexHullSeparatingAxis(const ConvexHullCollider* a, const ConvexHullCollider* b,
		const PolygonFace& aFace, int bVertexIndex) : BaseSeparatingAxis(dynamic_cast<const BaseCollider*>(a), dynamic_cast<const BaseCollider*>(b))
	{
		isVertexFace = true;
		vfInfo.face = aFace;
		vfInfo.vertexIndex = bVertexIndex;
	}

	ConvexHullConvexHullSeparatingAxis::ConvexHullConvexHullSeparatingAxis(const ConvexHullCollider* a, const ConvexHullCollider* b,
		int ea0, int ea1, int eb0, int eb1) :BaseSeparatingAxis(dynamic_cast<const BaseCollider*>(a), dynamic_cast<const BaseCollider*>(b))
	{
		isVertexFace = false;
		eeInfo.aEdgeIndices[0] = ea0; eeInfo.aEdgeIndices[1] = ea1;
		eeInfo.bEdgeIndices[0] = eb0; eeInfo.bEdgeIndices[1] = eb1;
	}


	ConvexHullConvexHullSeparatingAxis::~ConvexHullConvexHullSeparatingAxis()
	{}

	Eigen::Vector3d ConvexHullConvexHullSeparatingAxis::GetAxisDir() const
	{
		const ConvexHullCollider* a = dynamic_cast<const ConvexHullCollider*>(this->a);
		const ConvexHullCollider* b = dynamic_cast<const ConvexHullCollider*>(this->b);

		if (isVertexFace)
		{
			Eigen::Vector3d v0 = a->orientation * a->vertices[vfInfo.face[0]] + a->position;
			Eigen::Vector3d v1 = a->orientation * a->vertices[vfInfo.face[1]] + a->position;
			Eigen::Vector3d v2 = a->orientation * a->vertices[vfInfo.face[2]] + a->position;
			return (v2 - v1).cross(v0 - v1).normalized();
		}
		else
		{
			Eigen::Vector3d aEdgeDir = a->orientation *
				(a->vertices[eeInfo.aEdgeIndices[1]] -
					a->vertices[eeInfo.aEdgeIndices[0]]);
			Eigen::Vector3d bEdgeDir = b->orientation *
				(b->vertices[eeInfo.bEdgeIndices[1]] -
					b->vertices[eeInfo.bEdgeIndices[0]]);

			Eigen::Vector3d axisDir = aEdgeDir.cross(bEdgeDir).normalized();

			//flip axis dir if necessary
			if (axisDir.dot(GetReferencePlanePoint() - a->center) < 0)
				axisDir = -axisDir;

			return axisDir;
		}
	}

	Eigen::Vector3d ConvexHullConvexHullSeparatingAxis::GetReferencePlanePoint() const
	{
		if (isVertexFace)
		{
			const ConvexHullCollider* a = dynamic_cast<const ConvexHullCollider*>(this->a);
			return a->orientation * a->vertices[vfInfo.face[0]] + a->position;
		}
		else
		{
			const ConvexHullCollider* a = dynamic_cast<const ConvexHullCollider*>(this->a);
			return a->orientation * a->vertices[eeInfo.aEdgeIndices[0]] + a->position;
		}
	}
}