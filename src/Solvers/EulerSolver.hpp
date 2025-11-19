#include "OdeSolver.hpp"

struct EulerSolver : public OdeSolver
{
    virtual void Solve(double *y0, double *yend, size_t len, double t0, double t1, double dt, dydt dydtFunc) const override;
};