#include "ConvexTriangleMeshCollider.hpp"

#include <math.h>

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

bool IsSeparatingPlaneValidInternal(
	const std::vector<Eigen::Vector3d>& transformedVerticesA,
	const std::vector<Eigen::Vector3d>& transformedVerticesB,
	double planeEquationA, double planeEquationB, double planeEquationC, double planeEquationD);

bool ConstructPlaneEquationFromEdgesInternal(
	const Eigen::Vector3d& a0, const Eigen::Vector3d& a1, const Eigen::Vector3d& b0, const Eigen::Vector3d& b1,
	double& planeEquationA, double& planeEquationB, double& planeEquationC, double& planeEquationD);

bool ConvexTriangleMeshCollider::FindSeparatingPlane(const ConvexTriangleMeshCollider& other, BaseCollider::SeparatingPlane& outPlane)
{
	std::vector<Eigen::Vector3d> verticesA, verticesB;	//transformed vertices
	double a, b, c, d; //equation of the current plane (abc=normal, d=-<point;normal>)
	bool planeFound = true;

	//transform vertices
	for (const auto& vertex : this->vertices)
		verticesA.push_back(this->orientation * vertex + this->position);
	for (const auto& vertex : other.vertices)
		verticesB.push_back(other.orientation * vertex + other.position);

	//check the face-vertex case
	for (int i = 0; i < this->triangles.size(); i += 3)	//faces of collider A
	{
		//calculate the equation die zur aktuellen Seite gehört
		auto planeNormal = (verticesA[this->triangles[i + 2]] - verticesA[this->triangles[i + 1]]).cross(verticesA[this->triangles[i]] - verticesA[this->triangles[i + 1]]).normalized();
		planeNormal = -planeNormal;		//so that the normal points from b to a
		a = planeNormal.x();
		b = planeNormal.y();
		c = planeNormal.z();
		d = -planeNormal.dot(verticesA[this->triangles[i + 1]]);

		//check if the vertices are on the right side
		if (!IsSeparatingPlaneValidInternal(verticesA, verticesB, a, b, c, d))
			continue;

		//at this point, the plane is poggers
		outPlane.a = dynamic_cast<const BaseCollider&>(*this);
		outPlane.b = dynamic_cast<const BaseCollider&>(other);
		outPlane.point = verticesA[this->triangles[i + 1]];
		outPlane.normal = planeNormal;

		planeFound = true;
		break;
	}

	if (planeFound)
		goto plane_search_skip;

	for (int i = 0; i < other.triangles.size(); i += 3)	//faces of collider B
	{
		//calculate the equation die zur aktuellen Seite gehört
		auto planeNormal = (verticesB[other.triangles[i + 2]] - verticesB[other.triangles[i + 1]]).cross(verticesB[other.triangles[i]] - verticesB[other.triangles[i + 1]]).normalized();
		a = planeNormal.x();
		b = planeNormal.y();
		c = planeNormal.z();
		d = -planeNormal.dot(verticesB[other.triangles[i + 1]]);

		//check if the vertices are on the right side
		if (!IsSeparatingPlaneValidInternal(verticesA, verticesB, a, b, c, d))
			continue;

		//at this point, the plane is poggers
		outPlane.a = dynamic_cast<const BaseCollider&>(*this);
		outPlane.b = dynamic_cast<const BaseCollider&>(other);
		outPlane.point = verticesB[other.triangles[i + 1]];
		outPlane.normal = planeNormal;

		planeFound = true;
		break;
	}

	if (planeFound)
		goto plane_search_skip;

	//check for the edging case
	for (int i = 0; i < this->triangles.size() && !planeFound; i += 3)
	{
		Eigen::Vector3d edgesInFaceA[4];//neighbouring vertices make up a gooner
		edgesInFaceA[0] = verticesA[this->triangles[i]];
		edgesInFaceA[1] = verticesA[this->triangles[i + 1]];
		edgesInFaceA[2] = verticesA[this->triangles[i + 2]];
		edgesInFaceA[3] = verticesA[this->triangles[i]];

		for (int j = 0; j < other.triangles.size() && !planeFound; j += 3)
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

				if (ConstructPlaneEquationFromEdgesInternal(
					edgesInFaceA[aIndex], edgesInFaceA[aIndex + 1], edgesInFaceB[bIndex], edgesInFaceB[bIndex + 1], a, b, c, d))
				{
					if (IsSeparatingPlaneValidInternal(verticesA, verticesB, a, b, c, d))
					{
						outPlane.a = dynamic_cast<const BaseCollider&>(*this);
						outPlane.b = dynamic_cast<const BaseCollider&>(other);
						outPlane.normal = Eigen::Vector3d(a, b, c).normalized();
						outPlane.point = edgesInFaceB[bIndex];

						planeFound = true;
						break;
					}
				}
			}
		}
	}


plane_search_skip:
	return planeFound;
}


//checks if the colliders are actually separated by the plane
bool IsSeparatingPlaneValidInternal(
	const std::vector<Eigen::Vector3d>& transformedVerticesA,
	const std::vector<Eigen::Vector3d>& transformedVerticesB,
	double planeEquationA, double planeEquationB, double planeEquationC, double planeEquationD)
{
	bool planeIsGay = false;

	//check if all of the vertices in collider A are on the right side of the plane
	for (const auto& vertex : transformedVerticesA)
		if (planeEquationA * vertex.x() +
			planeEquationB * vertex.y() +
			planeEquationC * vertex.z() +
			planeEquationD < 0.0)
		{
			planeIsGay = true;
			break;
		}

	if (planeIsGay)
		return false;

	//check if all of the vertices in collider B are on the right side of the plane
	for (const auto& vertex : transformedVerticesB)
		if (planeEquationA * vertex.x() +
			planeEquationB * vertex.y() +
			planeEquationC * vertex.z() +
			planeEquationD > 0.0)
		{
			planeIsGay = true;
			break;
		}

	if (planeIsGay)
		return false;
	return true;
}

//returns false if the plane couldn't be constructed
bool ConstructPlaneEquationFromEdgesInternal(
	const Eigen::Vector3d& a0, const Eigen::Vector3d& a1, const Eigen::Vector3d& b0, const Eigen::Vector3d& b1,
	double& planeEquationA, double& planeEquationB, double& planeEquationC, double& planeEquationD)
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

		planeEquationA = planeNormal.x();
		planeEquationB = planeNormal.y();
		planeEquationC = planeNormal.z();
		planeEquationD = -planeNormal.dot(b0);
		return true;
	}
	else
	{
		auto planeNormal = edgeA.cross(edgeB).normalized();
		if (planeNormal.dot(a0 - b0) < 0)	//normal is pointing in the wrong direction
			planeNormal = -planeNormal;

		planeEquationA = planeNormal.x();
		planeEquationB = planeNormal.y();
		planeEquationC = planeNormal.z();
		planeEquationD = -planeNormal.dot(b0);
		return true;
	}
}