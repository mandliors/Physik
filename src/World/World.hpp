#pragma once

#include "Solvers/OdeSolver.hpp"
#include "Rigidbody/Rigidbody.hpp"
#include "Collision/Contact.hpp"

struct World
{
    void AddBody(Rigidbody *body);

    void FindAllCollisions(std::vector<Contact> &contacts) const;
    void Step(const OdeSolver &solver, double deltaTime);

    std::vector<Rigidbody *> rigidbodies;
};