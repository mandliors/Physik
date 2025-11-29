#include "LinAlg.hpp"

using namespace Eigen;

namespace LinAlg {
	LineSegment::LineSegment(const Vector3d& pointA, const Vector3d& pointB)
		:pointA(pointA), pointB(pointB)
	{}

	LineSegment LineSegment::ProjectOntoPlane(const Plane& plane) const
	{
		return LineSegment(plane.PointProjection(pointA), plane.PointProjection(pointB));
	}

	bool LineSegment::IntersectWithLineSegment(const LineSegment& a, const LineSegment& b, Eigen::Vector3d& outPoint)
	{
		Vector3d vA = a.pointB - a.pointA;
		Vector3d vB = b.pointB - b.pointA;
		const Vector3d& pA = a.pointA;
		const Vector3d& pB = b.pointA;

		double paramB =
			(pB.y() - pA.y() + pA.x() - pB.x()) /
			((vA.y() * vB.x()) / vA.x() - vB.y());
		double paramA = (vB.x() * paramB + pB.x() - pA.x()) / vA.x();

		//check if the points aren't beyond the endpoints
		if (paramA < 0 || paramA>1 || paramB < 0 || paramB>1)
			return false;

		//check if the intersection point actually exists
		double sugus = (vA.z() * paramA + pA.z()) - (vB.z() * paramB + pB.z());
		static constexpr double KINDA_ZERO = 0.000001;
		if (abs(sugus) > KINDA_ZERO)
			return false;

		//gg
		outPoint = vA * paramA + pA;
		return true;
	}


	Plane::Plane(const Vector3d& point, const Vector3d& normal)
		:point(point), normal(normal.normalized())
	{}

	Vector3d Plane::PointProjection(const Vector3d& point) const
	{
		return point - (normal.dot(point - this->point)) * normal;
	}


	bool Plane::IntersectWithLineSegment(const Plane& plane, const LineSegment& line, Vector3d& outPoint)
	{
		Vector3d v = line.pointB - line.pointA;
		const Vector3d& p = line.pointA;
		double a, b, c, d;

		//check if the line is parallel with the plane
		static constexpr double KINDA_ZERO = 0.000001;
		if (abs(v.dot(plane.normal)) < KINDA_ZERO)
			return false;

		//calculate plane equation
		a = plane.normal.x();
		b = plane.normal.y();
		c = plane.normal.z();
		d = -plane.normal.dot(plane.point);

		//calculate the intersection point
		double lineParam = -(a * p.x() + b * p.y() + c * p.z() + d) / (a * v.x() + b * v.y() + c * v.z());

		//check if the intersection point is on the plane
		if (lineParam < 0 || lineParam>1)
			return false;

		//return the point
		outPoint = lineParam * v + p;
		return true;
	}
}