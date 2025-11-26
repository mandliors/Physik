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

	BaseSeparatingAxis& BaseSeparatingAxis::operator=(const BaseSeparatingAxis& other)
	{
		this->a = other.a;
		this->b = other.b;

		return *this;
	}
}