#include "ConvexHullConvexHullSeparatingAxis.hpp"
#include "../../Collider/BaseCollider.hpp"
#include "../../Collider/ConvexHullCollider.hpp"
#include "../PolygonFace.hpp"

namespace Physik {

	ConvexHullConvexHullSeparatingAxis::ConvexHullConvexHullSeparatingAxis() :
		BaseSeparatingAxis(nullptr, nullptr), isVertexFace(false)
	{
		eeInfo.aEdgeIndices[0] = 0; eeInfo.aEdgeIndices[1] = 0;
		eeInfo.bEdgeIndices[0] = 0; eeInfo.bEdgeIndices[1] = 0;
	}

	ConvexHullConvexHullSeparatingAxis::ConvexHullConvexHullSeparatingAxis(const ConvexHullConvexHullSeparatingAxis& other)
		: BaseSeparatingAxis(other)
	{
		if (other.isVertexFace)
		{
			this->isVertexFace = true;
			this->vfInfo = other.vfInfo;
		}
		else
		{
			this->isVertexFace = false;
			this->eeInfo.aEdgeIndices[0] = other.eeInfo.aEdgeIndices[0];
			this->eeInfo.aEdgeIndices[1] = other.eeInfo.aEdgeIndices[1];
			this->eeInfo.bEdgeIndices[0] = other.eeInfo.bEdgeIndices[0];
			this->eeInfo.bEdgeIndices[1] = other.eeInfo.bEdgeIndices[1];
		}
	}

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

	ConvexHullConvexHullSeparatingAxis& ConvexHullConvexHullSeparatingAxis::operator=(const ConvexHullConvexHullSeparatingAxis& other)
	{
		this->BaseSeparatingAxis::operator=(other);

		if (other.isVertexFace)
		{
			this->isVertexFace = true;
			this->vfInfo = other.vfInfo;
		}
		else
		{
			this->isVertexFace = false;
			this->eeInfo = other.eeInfo;

		}

		return *this;
	}

	const PolygonFace& ConvexHullConvexHullSeparatingAxis::GetIncidentFace()
	{
		const ConvexHullCollider* b = dynamic_cast<const ConvexHullCollider*>(this->b);

		double minDot = DBL_MAX;
		int minFaceIndex = -1;
		Eigen::Vector3d refPlaneNormal = GetAxisDir();

		for (int i = 0; i < b->polygons.size(); i++)
		{
			const PolygonFace& face = b->polygons[i];

			if (face.size() < 3)
				continue;

			Eigen::Vector3d normal =
				(b->vertices[face[2]] - b->vertices[face[1]])
				.cross(b->vertices[face[0]] - b->vertices[face[1]])
				.normalized();

			double dot = normal.dot(refPlaneNormal);
			if (dot < minDot)
			{
				minDot = dot;
				minFaceIndex = i;
			}
		}

		if (minFaceIndex == -1)
			throw "Collider must have at least 1 polygon with a vertex count of at least 3";

		return b->polygons[minFaceIndex];
	}
}