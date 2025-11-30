#include "ConvexHullCollider.hpp"
#include "../Datatypes/SeparatingAxis/ConvexHullConvexHullSeparatingAxis.hpp"
#include "../../Math/LinAlg.hpp"

#include <math.h>
#include <float.h>

namespace Physik {
	//prototypes	----------------------------------------------------
	Eigen::Vector3d CalculateCenterInternal(const std::vector<Eigen::Vector3d>& vertices);

	bool GetAxisFromEdgesInternal(
		const Eigen::Vector3d& a0, const Eigen::Vector3d& a1, const Eigen::Vector3d& b0, const Eigen::Vector3d& b1,
		Eigen::Vector3d& outAxis);

	double GetDeepestPenetrator(
		const std::vector<Eigen::Vector3d>& transformedVertices,
		const Eigen::Vector3d& axis, const Eigen::Vector3d& referencePoint,
		int& deepestIndex);

	double CalculateFaceVertexSeparatingAxisInternal(
		const ConvexHullCollider* a, const ConvexHullCollider* b,
		const std::vector<Eigen::Vector3d>& transformedVerticesA,
		const std::vector<Eigen::Vector3d>& transformedVerticesB,
		ConvexHullConvexHullSeparatingAxis& outAxis);

	double CalculateEdgeEdgeSeparatingAxisInternal(
		const ConvexHullCollider* a, const ConvexHullCollider* b,
		const std::vector<Eigen::Vector3d>& transformedVerticesA,
		const std::vector<Eigen::Vector3d>& transformedVerticesB,
		ConvexHullConvexHullSeparatingAxis& outAxis);

	//implementations	---------------------------------------------

	ConvexHullCollider::ConvexHullCollider(const std::vector<Eigen::Vector3d>& vertices, const std::vector<long>& triangles)
		:BaseCollider(), vertices(vertices), triangles(triangles), center(CalculateCenterInternal(vertices))
	{
		CalculateBoundingSphere();
	}

	void ConvexHullCollider::CalculateBoundingSphere()
	{
		double maxSqrDistance = 0.0;
		for (const auto& vertex : vertices)
		{
			double temp = vertex.dot(vertex);
			if (temp > maxSqrDistance)
				maxSqrDistance = temp;
		}

		bounds.radius = sqrt(maxSqrDistance);
	}

	Eigen::Vector3d CalculateCenterInternal(const std::vector<Eigen::Vector3d>& vertices)
	{
		Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
		for (const Eigen::Vector3d& vertex : vertices)
			center += vertex;
		if (vertices.size() > 0)
			center /= (double)vertices.size();
		return center;
	}

	//SAT things	------------------------------------
	double ConvexHullCollider::FindSeparatingAxis(const ConvexHullCollider& b, ConvexHullConvexHullSeparatingAxis& outAxis)
	{
		std::vector<Eigen::Vector3d> transformedVerticesA;
		std::vector<Eigen::Vector3d> transformedVerticesB;

		ConvexHullConvexHullSeparatingAxis axisAB; double distanceAB;
		ConvexHullConvexHullSeparatingAxis axisBA; double distanceBA;
		ConvexHullConvexHullSeparatingAxis axisEdgeAB; double distanceEdgeAB;


		//calculate transformed vertices
		for (const Eigen::Vector3d& vertex : this->vertices)
			transformedVerticesA.push_back(this->orientation * vertex + this->position);

		for (const Eigen::Vector3d& vertex : b.vertices)
			transformedVerticesB.push_back(b.orientation * vertex + b.position);

		//get the separating axes
		CalculateFaceVertexSeparatingAxisInternal(this, &b, transformedVerticesA, transformedVerticesB, axisAB);
		CalculateFaceVertexSeparatingAxisInternal(&b, this, transformedVerticesB, transformedVerticesA, axisBA);
		CalculateEdgeEdgeSeparatingAxisInternal(this, &b, transformedVerticesA, transformedVerticesB, axisEdgeAB);

		ConvexHullConvexHullSeparatingAxis& maxAxis = axisAB;
		double maxDistance = distanceAB;

		if (distanceBA > maxDistance)
		{
			maxAxis = axisBA;
			maxDistance = distanceBA;
		}
		if (distanceEdgeAB > maxDistance)
		{
			maxAxis = axisEdgeAB;
			maxDistance = distanceEdgeAB;
		}

		if (maxDistance != DBL_MIN)
			outAxis = maxAxis;
		return maxDistance;
	}

	//returns the axis with the maximum separation where the reference plane is given by a face of a
	//returns DBL_MIN if no separating axis is found (in this case, outAxis is not overwritten)
	double CalculateFaceVertexSeparatingAxisInternal(
		const ConvexHullCollider* a, const ConvexHullCollider* b,
		const std::vector<Eigen::Vector3d>& transformedVerticesA,
		const std::vector<Eigen::Vector3d>& transformedVerticesB,
		ConvexHullConvexHullSeparatingAxis& outAxis)
	{
		ConvexHullConvexHullSeparatingAxis currentAxis;
		double currentDistance = DBL_MIN;

		for (const PolygonFace& face : a->polygons)
		{
			Eigen::Vector3d tempNormal =
				(transformedVerticesA[face[2]] - transformedVerticesA[face[1]]).cross(
					transformedVerticesA[face[0]] - transformedVerticesA[face[1]]
				);
			int penetratorIndex = -1;
			double penetratorDistance = DBL_MAX;

			penetratorDistance = GetDeepestPenetrator(
				transformedVerticesB,
				tempNormal,
				transformedVerticesA[face[0]],
				penetratorIndex);

			if (penetratorDistance > currentDistance)
			{
				currentDistance = penetratorDistance;
				currentAxis = ConvexHullConvexHullSeparatingAxis(a, b, face, penetratorIndex);
			}
		}

		if (currentDistance > DBL_MIN)
			outAxis = currentAxis;
		return currentDistance;
	}

	//returns the axis with the maximum separation where the reference plane is given by two gooners
	//the axis power points from a to b
	//returns DBL_MIN if no separating axis is found (in this case, outAxis is not overwritten)
	double CalculateEdgeEdgeSeparatingAxisInternal(
		const ConvexHullCollider* a, const ConvexHullCollider* b,
		const std::vector<Eigen::Vector3d>& transformedVerticesA,
		const std::vector<Eigen::Vector3d>& transformedVerticesB,
		ConvexHullConvexHullSeparatingAxis& outAxis)
	{
		Eigen::Vector3d aCenterTransformed = a->orientation * a->center + a->position;
		ConvexHullConvexHullSeparatingAxis currentAxis;
		double currentDistance = DBL_MIN;

		for (const PolygonFace& aFace : a->polygons)
		{
			for (const PolygonFace& bFace : b->polygons)
			{
				for (int i = 0; i < aFace.size(); i++)	//edges in face a
				{
					for (int j = 0; j < bFace.size(); j++)	//edges in face b
					{
						Eigen::Vector3d tempNormal;

						//check if they can edge
						if (!GetAxisFromEdgesInternal(
							transformedVerticesA[aFace[i]], transformedVerticesA[aFace[(i + 1) % aFace.size()]],
							transformedVerticesB[bFace[j]], transformedVerticesB[bFace[(j + 1) % bFace.size()]],
							tempNormal
						))
							continue;

						ConvexHullConvexHullSeparatingAxis tempAxis =
							ConvexHullConvexHullSeparatingAxis(a, b, aFace[i], aFace[(i + 1) % aFace.size()], bFace[j], bFace[(j + 1) % bFace.size()]);

						int nigga;
						double maxDistance = GetDeepestPenetrator(transformedVerticesB, tempNormal, tempAxis.GetReferencePlanePoint(), nigga);

						if (maxDistance > currentDistance)
						{
							currentDistance = maxDistance;
							currentAxis = tempAxis;
						}
					}
				}
			}
		}

		if (currentDistance > DBL_MIN)
			outAxis = currentAxis;
		return currentDistance;
	}

	//returns false if there was no axis to be found (in this case, outAxis shall not be molested)
	bool GetAxisFromEdgesInternal(
		const Eigen::Vector3d& a0, const Eigen::Vector3d& a1, const Eigen::Vector3d& b0, const Eigen::Vector3d& b1,
		const Eigen::Vector3d& aCenter, Eigen::Vector3d& outAxis)
	{
		constexpr double MIN_VEC_SQRLENGTH = 0.0000001;
		auto edgeA = a1 - a0;
		auto edgeB = b1 - b0;

		if (edgeA.dot(edgeA) < MIN_VEC_SQRLENGTH || edgeB.dot(edgeB) < MIN_VEC_SQRLENGTH)//at least one of the edges don't have golden gooner rank
			return false;

		if (edgeA.cross(edgeB).squaredNorm() < MIN_VEC_SQRLENGTH)//the edges are perfectly parallel
		{
			auto planeNormal = edgeB.cross(a0 - b0).cross(edgeB);
			if (planeNormal.dot(a0 - aCenter) < 0)	//normal is pointing in the wrong direction
				planeNormal = -planeNormal;

			outAxis = planeNormal.normalized();
			return true;
		}
		else
		{
			auto planeNormal = edgeA.cross(edgeB).normalized();
			if (planeNormal.dot(a0 - aCenter) < 0)	//normal is pointing in the wrong direction
				planeNormal = -planeNormal;

			outAxis = planeNormal.normalized();
			return true;
		}
	}

	double GetDeepestPenetrator(
		const std::vector<Eigen::Vector3d>& transformedVertices,
		const Eigen::Vector3d& axis, const Eigen::Vector3d& referencePoint,
		int& deepestIndex)
	{
		Eigen::Vector3d normAxis = axis.normalized();
		double planeEquationConstant = -normAxis.dot(referencePoint);

		int minIndex = -1;
		int minDistance = DBL_MAX;

		for (int i = 0; i < transformedVertices.size(); i++)
		{
			double distance = normAxis.dot(transformedVertices[i]) + planeEquationConstant;
			if (distance < minDistance)
			{
				minIndex = i;
				minDistance = distance;
			}
		}

		if (minIndex != -1)
			deepestIndex = minIndex;
		return minDistance;
	}


	//collisions ----------------------------------------------

	bool ConvexHullCollider::CollideWithConvexHull(const ConvexHullCollider& other, std::vector<MeshContact>& outContactManifold)
	{
		ConvexHullConvexHullSeparatingAxis axis;
		LinAlg::Plane referencePlane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

		std::vector<MeshContact> contacts;

		//get separating axis
		double separation = FindSeparatingAxis(other, axis);
		if (DBL_MIN == separation)	//axis doesnt exist
			return false;
		if (separation >= 0)	//no collision
			return false;

		//handle edging and gooning separately
		if (axis.isVertexFace)
		{
			const ConvexHullCollider* a = dynamic_cast<const ConvexHullCollider*>(axis.a);
			const ConvexHullCollider* b = dynamic_cast<const ConvexHullCollider*>(axis.b);

			const PolygonFace& incidentFace = axis.GetIncidentFace();

			std::vector<LinAlg::Plane> referenceFaceSidePlanes;		//the sides planes of the reference face
			std::vector<LinAlg::LineSegment> incidentFaceEdges;		//transformed incident face edges


			//calculate the reference face side planes
			for (int i = 0; i < axis.vfInfo.face.size(); i++)
			{
				Eigen::Vector3d edgeA = a->orientation * a->vertices[axis.vfInfo.face[i]] + a->position;
				Eigen::Vector3d edgeB = a->orientation * a->vertices[axis.vfInfo.face[i + 1]] + a->position;

				Eigen::Vector3d sidePlaneNormal = axis.GetAxisDir().cross(edgeB - edgeA);
				referenceFaceSidePlanes.push_back(LinAlg::Plane(edgeA, sidePlaneNormal));
			}

			//calculate the incident face edges
			for (int i = 0; i < incidentFace.size(); i++)
				incidentFaceEdges.push_back(LinAlg::LineSegment(
					b->orientation * b->vertices[incidentFace[i]] + b->position,
					b->orientation * b->vertices[incidentFace[i + 1]] + b->position));

			//clip the incident edges against the reference side planes
			for (const LinAlg::LineSegment& edge : incidentFaceEdges)
			{
				for (const LinAlg::Plane& plane : referenceFaceSidePlanes)
				{
					Eigen::Vector3d contactPoint;
					if (LinAlg::Plane::IntersectWithLineSegment(plane, edge, contactPoint))//intersection between the edge and the reference plane
						contacts.push_back(MeshContact(a, b, contactPoint, axis.GetAxisDir()));
				}
			}
		}
		else
		{
			const ConvexHullCollider* a = dynamic_cast<const ConvexHullCollider*>(axis.a);
			const ConvexHullCollider* b = dynamic_cast<const ConvexHullCollider*>(axis.b);

			LinAlg::LineSegment edgeA = LinAlg::LineSegment(
				a->orientation * a->vertices[axis.eeInfo.aEdgeIndices[1]] + a->position,
				a->orientation * a->vertices[axis.eeInfo.aEdgeIndices[0]] + a->position
			);
			LinAlg::LineSegment edgeB = LinAlg::LineSegment(
				b->orientation * b->vertices[axis.eeInfo.bEdgeIndices[1]] + b->position,
				b->orientation * b->vertices[axis.eeInfo.bEdgeIndices[0]] + b->position
			);

			LinAlg::Plane referencePlane(axis.GetReferencePlanePoint(), axis.GetAxisDir());

			//project the edges onto the reference plane
			edgeA = edgeA.ProjectOntoPlane(referencePlane);
			edgeB = edgeB.ProjectOntoPlane(referencePlane);

			//get the contact point
			Eigen::Vector3d contactPoint;
			if (LinAlg::LineSegment::IntersectWithLineSegment(edgeA, edgeB, contactPoint))
				contacts.push_back(MeshContact(a, b, contactPoint, axis.GetAxisDir()));
		}

		//simplify the contact manifold if necessary
	}
}