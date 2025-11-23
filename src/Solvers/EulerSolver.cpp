#include "EulerSolver.hpp"

namespace Physik::Solvers
{
    void EulerSolver::Solve(double *y0, double *yend, size_t len, double t0, double t1, double dt, dydt dydtFunc) const
    {
        std::memset(yend, 0, len * sizeof(double));
        for (double t = t0; t < t1; t += dt)
        {
            dydtFunc(t, y0, yend);
            for (size_t i = 0; i < len; ++i)
                yend[i] = y0[i] + dt * yend[i];
            std::memcpy(y0, yend, len * sizeof(double));
        }
    }
}