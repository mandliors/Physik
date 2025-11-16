#include "BoxCollider.hpp"
#include "SphereCollider.hpp"
#include "CollisionInfo.hpp"

CollisionInfo SphereCollider::CollideWith(const Collider* other) const
{
	return other->CollideWithSphere(this);
}
CollisionInfo SphereCollider::CollideWithBox(const BoxCollider* other) const
{
	return CollisionInfo{
		.colliderA = *this,
		.colliderB = *other
	};
}
CollisionInfo SphereCollider::CollideWithSphere(const SphereCollider* other) const
{
	return CollisionInfo{
		.colliderA = *this,
		.colliderB = *other
	};
}