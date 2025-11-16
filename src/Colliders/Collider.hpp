#pragma once

#include <glm/glm.hpp>

struct BoxCollider;
struct SphereCollider;

struct CollisionInfo;

struct Collider
{
	virtual CollisionInfo CollideWith(const Collider* other) const = 0;
	virtual CollisionInfo CollideWithBox(const BoxCollider* other) const = 0;
	virtual CollisionInfo CollideWithSphere(const SphereCollider* other) const = 0;
};