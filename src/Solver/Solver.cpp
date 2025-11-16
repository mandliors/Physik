#include "Solver/Solver.hpp"

void Solver::Solve(const std::vector<std::reference_wrapper<Rigidbody>>& bodies) const
{
	for (Rigidbody& body : bodies)
		body.ClearForces();

	for (size_t i = 0; i < bodies.size(); i++)
		for (size_t j = i + 1; j < bodies.size(); j++)
			bodies[i].get().CollideWith(bodies[j].get());

	for (Rigidbody& body : bodies)
		body.ApplyForces();
}