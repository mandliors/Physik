#include "BoxCollider.hpp"
#include "SphereCollider.hpp"
#include "CollisionInfo.hpp"

CollisionInfo BoxCollider::CollideWith(const Collider* other) const
{
	return other->CollideWithBox(this);
}
CollisionInfo BoxCollider::CollideWithBox(const BoxCollider* other) const
{
	return CollisionInfo{
		.colliderA = *this,
		.colliderB = *other
	};
}
CollisionInfo BoxCollider::CollideWithSphere(const SphereCollider* other) const
{
	return CollisionInfo{
		.colliderA = *this,
		.colliderB = *other
	};
}