#pragma once

#include "Rigidbody/Rigidbody.hpp"

#include <vector>

struct Solver
{
	void Solve(const std::vector<std::reference_wrapper<Rigidbody>>& bodies) const;
};