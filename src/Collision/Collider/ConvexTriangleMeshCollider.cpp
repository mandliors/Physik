#include "ConvexTriangleMeshCollider.hpp"

#include <math.h>
#include <float.h>

ConvexTriangleMeshCollider::ConvexTriangleMeshCollider(const std::vector<Eigen::Vector3d>& vertices, const std::vector<long>& triangles)
	:BaseCollider(), vertices(vertices), triangles(triangles)
{
	CalculateBoundingSphere();
}

void ConvexTriangleMeshCollider::CalculateBoundingSphere()
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

double CalculateAxisSeparation(
	const std::vector<Eigen::Vector3d>& transformedVerticesA,
	const std::vector<Eigen::Vector3d>& transformedVerticesB,
	Eigen::Vector3d axis, int& closestIndexA, int& closestIndexB);

bool GetAxisFromEdgesInternal(
	const Eigen::Vector3d& a0, const Eigen::Vector3d& a1, const Eigen::Vector3d& b0, const Eigen::Vector3d& b1,
	Eigen::Vector3d& outAxis);

double ConvexTriangleMeshCollider::FindSeparatingAxis(const ConvexTriangleMeshCollider& other, BaseCollider::SeparatingAxis& outPlane)
{
	std::vector<Eigen::Vector3d> verticesA, verticesB;	//transformed vertices
	double maxSeparation = DBL_MIN;
	BaseCollider::SeparatingAxis currentAxis;

	if (this->triangles.size() == 0 || other.triangles.size() == 0)
		return DBL_MIN;

	//transform vertices
	for (const auto& vertex : this->vertices)
		verticesA.push_back(this->orientation * vertex + this->position);
	for (const auto& vertex : other.vertices)
		verticesB.push_back(other.orientation * vertex + other.position);

	//check the face-vertex case
	for (int i = 0; i < this->triangles.size(); i += 3)	//faces of collider A
	{
		//calculate the axis die zur aktuellen Seite gehört
		auto planeNormal = (verticesA[this->triangles[i + 2]] - verticesA[this->triangles[i + 1]]).cross(verticesA[this->triangles[i]] - verticesA[this->triangles[i + 1]]).normalized();
		int closestIndexA = -1, closestIndexB = -1;

		//check if the axis is based
		double separation = CalculateAxisSeparation(verticesA, verticesB, planeNormal, closestIndexA, closestIndexB);
		if (separation > maxSeparation)
		{
			maxSeparation = separation;
			currentAxis = SeparatingAxis(&other, this, planeNormal, closestIndexB, i, i + 1, i + 2);
		}
	}


	for (int i = 0; i < other.triangles.size(); i += 3)	//faces of collider B
	{
		//calculate the axis die zur aktuellen Seite gehört
		auto planeNormal = (verticesB[other.triangles[i + 2]] - verticesB[other.triangles[i + 1]]).cross(verticesB[other.triangles[i]] - verticesB[other.triangles[i + 1]]).normalized();
		int closestIndexA = -1, closestIndexB = -1;

		//check if the axis is based
		double separation = CalculateAxisSeparation(verticesA, verticesB, planeNormal, closestIndexA, closestIndexB);
		if (separation > maxSeparation)
		{
			maxSeparation = separation;
			currentAxis = SeparatingAxis(this, &other, planeNormal, closestIndexA, i, i + 1, i + 2);
		}
	}

	//check for the edging case
	for (int i = 0; i < this->triangles.size(); i += 3)
	{
		Eigen::Vector3d edgesInFaceA[4];//neighbouring vertices make up a gooner
		edgesInFaceA[0] = verticesA[this->triangles[i]];
		edgesInFaceA[1] = verticesA[this->triangles[i + 1]];
		edgesInFaceA[2] = verticesA[this->triangles[i + 2]];
		edgesInFaceA[3] = verticesA[this->triangles[i]];

		for (int j = 0; j < other.triangles.size(); j += 3)
		{
			Eigen::Vector3d edgesInFaceB[4];//neighbouring vertices make up a gooner
			edgesInFaceB[0] = verticesB[other.triangles[j]];
			edgesInFaceB[1] = verticesB[other.triangles[j + 1]];
			edgesInFaceB[2] = verticesB[other.triangles[j + 2]];
			edgesInFaceB[3] = verticesB[other.triangles[j]];

			for (int k = 0; k < 9; k++)
			{
				int aIndex = k / 3;
				int bIndex = k % 3;
				Eigen::Vector3d axis;

				if (GetAxisFromEdgesInternal(
					edgesInFaceA[aIndex], edgesInFaceA[aIndex + 1],
					edgesInFaceB[bIndex], edgesInFaceB[bIndex + 1],
					axis))	//axis exists
				{
					int closestA, closestB;
					double separation = CalculateAxisSeparation(verticesA, verticesB, axis, closestA, closestB);
					if (separation > maxSeparation)
					{
						maxSeparation = separation;
						currentAxis = SeparatingAxis(
							this, &other, axis,
							this->triangles[i + aIndex], this->triangles[i + (aIndex + 1) % 3],
							other.triangles[j + bIndex], other.triangles[j + (bIndex + 1) % 3], 69);
					}
				}
			}
		}
	}

	return maxSeparation;
}


//calculates if the colliders are actually separated along the axis
//returns the distance between the colliders along the axis (negative if there is a penetration)
double CalculateAxisSeparation(
	const std::vector<Eigen::Vector3d>& transformedVerticesA,
	const std::vector<Eigen::Vector3d>& transformedVerticesB,
	Eigen::Vector3d axis, int& closestIndexA, int& closestIndexB)
{
	double minA = DBL_MAX, maxA = DBL_MIN;
	double minB = DBL_MAX, maxB = DBL_MIN;
	int minIndexA = -1, maxIndexA = -1;
	int minIndexB = -1, maxIndexB = -1;
	auto normalizedAxis = axis.normalized();

	if (transformedVerticesA.size() == 0 || transformedVerticesB.size() == 0)
		return 0.0;

	//calculate the min projection of the vertices in collider a
	for (int i = 0; i < transformedVerticesA.size(); i++)
	{
		double proj = transformedVerticesA[i].dot(normalizedAxis);
		if (proj < minA)
		{
			minA = proj;
			minIndexA = i;
		}
		if (proj > maxA)
		{
			maxA = proj;
			maxIndexA = i;
		}
	}

	//calculate the min projection of the vertices in collider b
	for (int i = 0; i < transformedVerticesB.size(); i++)
	{
		double proj = transformedVerticesB[i].dot(normalizedAxis);
		if (proj < minB)
		{
			minB = proj;
			minIndexB = i;
		}
		if (proj > maxB)
		{
			maxB = proj;
			maxIndexB = i;
		}
	}

	//calculate the separation
	double separation = minB - maxA;
	if (minA - maxB > separation)
	{
		separation = minA - maxB;
		closestIndexA = minIndexA;
		closestIndexB = maxIndexB;
	}
	else
	{
		closestIndexA = maxIndexA;
		closestIndexB = minIndexB;
	}

	return separation;
}

//returns false if there was no axis to be found (in this case, outAxis shall not be molested)
bool GetAxisFromEdgesInternal(
	const Eigen::Vector3d& a0, const Eigen::Vector3d& a1, const Eigen::Vector3d& b0, const Eigen::Vector3d& b1,
	Eigen::Vector3d& outAxis)
{
	constexpr double MIN_VEC_SQRLENGTH = 0.0000001;
	auto edgeA = a1 - a0;
	auto edgeB = b1 - b0;

	if (edgeA.dot(edgeA) < MIN_VEC_SQRLENGTH || edgeB.dot(edgeB) < MIN_VEC_SQRLENGTH)//at least one of the edges don't have golden gooner rank
		return false;

	if (edgeA.cross(edgeB).squaredNorm() < MIN_VEC_SQRLENGTH)//the edges are perfectly parallel
	{
		auto planeNormal = edgeB.cross(a0 - b0).cross(edgeB);
		if (planeNormal.dot(a0 - b0) < 0)	//normal is pointing in the wrong direction
			planeNormal = -planeNormal;

		outAxis = planeNormal.normalized();
		return true;
	}
	else
	{
		auto planeNormal = edgeA.cross(edgeB).normalized();
		if (planeNormal.dot(a0 - b0) < 0)	//normal is pointing in the wrong direction
			planeNormal = -planeNormal;

		outAxis = planeNormal.normalized();
		return true;
	}
}