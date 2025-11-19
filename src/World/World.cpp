#include "World.hpp"
#include "../Math/Math.hpp"

void World::Step(const OdeSolver &solver, double deltaTime)
{
    // for (size_t i = 0; i < rigidbodies.size(); i++)
    //     for (size_t j = i + 1; j < rigidbodies.size(); j++)
    //         rigidbodies[i]->CollideWith(*rigidbodies[j]);

    std::vector<Contact> contacts;
    // find contacts

    // find and solve all colliding contacts
    FindAllCollisions(contacts);

    // solve resting contacts
    std::vector<Contact> restingContacts;
    std::copy_if(contacts.begin(), contacts.end(), std::back_inserter(restingContacts), [](Contact c)
                 { return c.IsResting(); });
    ComputeContactForces(restingContacts, deltaTime);

    // Integrate the state of each rigidbody
    for (Rigidbody *body : rigidbodies)
    {
        body->Step(solver, deltaTime);

        body->CalculateDerivedQuantities();
        body->ClearForces();
    }
}

void World::FindAllCollisions(std::vector<Contact> &contacts) const
{
    static constexpr float RESTITUTION_COEFFICIENT = 0.5f;
    bool collisionDetected = true;

    do
    {
        collisionDetected = false;
        for (Contact &contact : contacts)
        {
            if (contact.IsColliding())
            {
                contact.DoCollisionResponse(RESTITUTION_COEFFICIENT);
                collisionDetected = true;
                // OdeDiscontinuous();
            }
        }
    } while (collisionDetected);
}