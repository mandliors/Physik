#include "BaseSeparatingAxis.hpp"

namespace Physik {

	BaseSeparatingAxis::BaseSeparatingAxis()
		:a(nullptr), b(nullptr)
	{}

	BaseSeparatingAxis::BaseSeparatingAxis(const BaseCollider* a, const BaseCollider* b)
		: a(a), b(b)
	{ }

	BaseSeparatingAxis::~BaseSeparatingAxis()
	{}
}