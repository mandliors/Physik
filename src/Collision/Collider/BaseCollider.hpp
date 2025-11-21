#pragma once

#include <Eigen/Dense>

class BaseCollider {
public:
	class BoundingSphere {
	public:
		double radius;

		BoundingSphere();
	};

	class SeparatingAxis {
	public:
		const BaseCollider* a, * b; //in case of vertex-face contact, b contains the face
		Eigen::Vector3d axisDir;	//from b to a
		bool isVertexFace;
		union {
			struct {
				int vertexIndex;
				int faceIndices[3];
			} vfInfo;
			struct {
				int edge0Indices[2];
				int edge1Indices[2];
			} eeInfo;
		};

		SeparatingAxis();

		SeparatingAxis(const BaseCollider* a, const BaseCollider* b, const Eigen::Vector3d& axisDir,
			int vertexIndex, int faceIndex0, int faceIndex1, int faceIndex2);

		SeparatingAxis(const BaseCollider* a, const BaseCollider* b, const Eigen::Vector3d& axisDir,
			int edge0Index0, int edge0Index1, int edge1Index0, int edge1Index1, int edgeEdgeDummy);
	};

	BoundingSphere bounds;
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

	BaseCollider();

	virtual void CalculateBoundingSphere() = 0;
};