#include "World.hpp"
#include "../Math/Math.hpp"

static std::vector<Contact> FindAllContacts()
{
    return {};
}

void World::AddBody(Rigidbody *body)
{
    rigidbodies.push_back(body);
}
void World::Step(const OdeSolver &solver, double deltaTime)
{
    std::vector<Contact> contacts;

    for (Rigidbody *body : rigidbodies)
    {
        body->ApplyForces();

        // -------- COLLISIONS --------
        {
            contacts = FindAllContacts();

            // solve colliding contacts
            SolveCollidingContacts(contacts);

            // solve resting contacts
            std::vector<Contact> restingContacts;
            std::copy_if(contacts.begin(), contacts.end(), std::back_inserter(restingContacts), [](Contact c)
                         { return c.GetState() == Contact::State::Resting; });
            SolveRestingContacts(restingContacts, deltaTime);
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

void World::SolveCollidingContacts(std::vector<Contact> &contacts) const
{
    static constexpr float RESTITUTION_COEFFICIENT = 0.5f;
    bool collisionDetected = true;

    do
    {
        collisionDetected = false;
        for (Contact &contact : contacts)
        {
            if (contact.GetState() == Contact::State::Colliding)
            {
                contact.DoCollisionResponse(RESTITUTION_COEFFICIENT);
                collisionDetected = true;
            }
        }
    } while (collisionDetected);
}