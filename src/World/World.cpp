#include "World.hpp"
#include "../Math/Math.hpp"

static std::vector<Physik::Contact> FindAllContacts()
{
    return {};
}

namespace Physik
{
    void World::AddBody(Rigidbody *body)
    {
        rigidbodies.push_back(body);
    }
    void World::Step(const Solvers::OdeSolver &solver, double deltaTime)
    {
        std::vector<Contact> contacts;

        for (Rigidbody *body : rigidbodies)
        {
            body->ApplyForces();

            // -------- COLLISIONS --------
            {
                contacts = FindAllContacts();

                // solve colliding contacts
                Math::SolveCollidingContacts(contacts);

                // solve resting contacts
                std::vector<Contact> restingContacts;
                std::copy_if(contacts.begin(), contacts.end(), std::back_inserter(restingContacts), [](Contact c)
                             { return c.GetState() == Contact::State::Resting; });
                Math::SolveRestingContacts(restingContacts, deltaTime);
            }
            // ----------------------------

            // -------- INTEGRATION --------
            {
                body->CalculateDerivedQuantities();
                body->Step(solver, deltaTime);
            }
            // -----------------------------

            body->ClearForces();
        }
    }
}