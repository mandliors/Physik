#pragma once

#include <Eigen/Dense>

namespace LinAlg {
	class LineSegment;
	class Plane;

	class LineSegment {
	public:
		const Eigen::Vector3d pointA, pointB;

		LineSegment(const Eigen::Vector3d& pointA, const Eigen::Vector3d& pointB);

		LineSegment ProjectOntoPlane(const Plane& plane) const;

		//returns true if there is an intersection
		static bool IntersectWithLineSegment(const LineSegment& a, const LineSegment& b, Eigen::Vector3d& outPoint);
	};

	class Plane {
	public:
		const Eigen::Vector3d point;
		const Eigen::Vector3d normal;	//normalized automatically

		Plane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal);

		Eigen::Vector3d PointProjection(const Eigen::Vector3d& point) const;

		//returns true if there is an intersection
		static bool IntersectWithLineSegment(const Plane& plane, const LineSegment& line, Eigen::Vector3d& outPoint);
	};
}