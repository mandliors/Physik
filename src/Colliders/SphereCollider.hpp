#pragma once

#include "Collider.hpp"

struct SphereCollider : public Collider
{
	CollisionInfo CollideWith(const Collider& other) const override;
	CollisionInfo CollideWithBox(const BoxCollider& other) const override;
	CollisionInfo CollideWithSphere(const SphereCollider& other) const override;
};