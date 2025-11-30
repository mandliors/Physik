#include "Contact.hpp"

namespace Physik
{
	Contact::Contact(Rigidbody& a, Rigidbody& b)
		: a(a), b(b), type(Type::VertexFace)
	{
	}

	Contact::State Contact::GetState() const
	{
		static constexpr double COLLISION_EPSILON = 0.01;

		auto paVel = a.GetPointVelocity(position);
		auto pbVel = b.GetPointVelocity(position);
		auto relVel = pbVel - paVel;
		double relVelAlongNormal = relVel.dot(normal);

		if (relVelAlongNormal > COLLISION_EPSILON)
			return State::Separating;
		else if (relVelAlongNormal > -COLLISION_EPSILON)
			return State::Resting;
		else
			return State::Colliding;
	}

	void Contact::DoCollisionResponse(float restitutionCoefficient)
	{
		auto paVel = a.GetPointVelocity(position);
		auto pbVel = b.GetPointVelocity(position);
		auto relVel = pbVel - paVel;

		auto ra = position - a.state.position;
		auto rb = position - b.state.position;

		double velAlongNormal = normal.dot(relVel);
		double numerator = -(1.0 + restitutionCoefficient) * velAlongNormal;

		double term1 = a.constants.inverseMass;
		double term2 = b.constants.inverseMass;
		double term3 = normal.dot((a.derived.inverseInertia * (ra.cross(normal))).cross(ra));
		double term4 = normal.dot((b.derived.inverseInertia * (rb.cross(normal))).cross(rb));

		double j = numerator / (term1 + term2 + term3 + term4);
		auto impulse = j * normal;

		a.state.linearMomentum += impulse;
		b.state.linearMomentum -= impulse;
		a.state.angularMomentum += ra.cross(impulse);
		b.state.angularMomentum -= rb.cross(impulse);

		// TODO: recalculating velocty and angularvelocity might be enough
		a.CalculateDerivedQuantities();
		b.CalculateDerivedQuantities();
	}

	Eigen::Vector3d Contact::ComputeNDot() const
	{
		if (type == Type::VertexFace)
			return b.derived.angularVelocity.cross(normal);

		auto eadot = a.derived.angularVelocity.cross(edgeA);
		auto ebdot = b.derived.angularVelocity.cross(edgeB);

		auto n = edgeA.cross(edgeB);
		double nLength = n.norm();
		n /= nLength;

		auto z = eadot.cross(edgeB) + edgeA.cross(ebdot);

		return (z - ((z.dot(n)) * n)) / nLength;
	}
}