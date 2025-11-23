#pragma once

#include <Eigen/Dense>

#include <functional>
#include <cstddef>

namespace Physik::Solvers
{
	struct OdeSolver
	{
		using dydt = std::function<void(double, double *, double *)>;
		virtual void Solve(double *y0, double *yend, size_t len, double t0, double t1, double dt, dydt dydtFunc) const = 0;
	};
}