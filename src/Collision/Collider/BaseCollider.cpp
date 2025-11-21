#include "BaseCollider.hpp"

BaseCollider::BoundingSphere::BoundingSphere() :radius(0.0)
{}

BaseCollider::SeparatingAxis::SeparatingAxis()
	:a(nullptr), b(nullptr), axisDir(Eigen::Vector3d(0.0, 0.0, 0.0))
{}

BaseCollider::SeparatingAxis::SeparatingAxis(const BaseCollider* a, const BaseCollider* b, const Eigen::Vector3d& axisDir,
	int vertexIndex, int faceIndex0, int faceIndex1, int faceIndex2)
	: a(a), b(b), axisDir(axisDir.normalized()), isVertexFace(true)
{
	vfInfo.vertexIndex = vertexIndex;
	vfInfo.faceIndices[0] = faceIndex0;
	vfInfo.faceIndices[1] = faceIndex1;
	vfInfo.faceIndices[2] = faceIndex2;
}

BaseCollider::SeparatingAxis::SeparatingAxis(const BaseCollider* a, const BaseCollider* b, const Eigen::Vector3d& axisDir,
	int edge0Index0, int edge0Index1, int edge1Index0, int edge1Index1, int edgeEdgeDummy)
	: a(a), b(b), axisDir(axisDir.normalized()), isVertexFace(false)
{
	eeInfo.edge0Indices[0] = edge0Index0;
	eeInfo.edge0Indices[1] = edge0Index1;
	eeInfo.edge1Indices[0] = edge1Index0;
	eeInfo.edge1Indices[1] = edge1Index1;
}

BaseCollider::BaseCollider()
{}