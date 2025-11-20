#pragma once

#include "Solvers/OdeSolver.hpp"
#include "Rigidbody/Rigidbody.hpp"
#include "Collision/Contact.hpp"

struct World
{
    void AddBody(Rigidbody *body);
    void Step(const OdeSolver &solver, double deltaTime);
    void SolveCollidingContacts(std::vector<Contact> &contacts) const;

    std::vector<Rigidbody *> rigidbodies;
};