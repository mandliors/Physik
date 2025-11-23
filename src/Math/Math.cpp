#include "Math/Math.hpp"
#include "Math/eiquadprog.hpp"

#include <array>

namespace Physik::Math
{
    void CalculateMassProperties(const Mesh &mesh, double density, Rigidbody::MassProperties &massProps)
    {
        // small value to avoid division by zero
        static constexpr double DET_EPS = 1e-12;

        // integration multipliers for the 10 integrals
        static constexpr auto multipliers = std::array<double, 10>{
            1.0 / 6.0,
            1.0 / 24.0,
            1.0 / 24.0,
            1.0 / 24.0,
            1.0 / 60.0,
            1.0 / 60.0,
            1.0 / 60.0,
            1.0 / 120.0,
            1.0 / 120.0,
            1.0 / 120.0};

        // array to hold the 10 integrals needed for mass properties
        auto integrals = std::array<double, 10>{0};

        // helper function to compute sub-expressions
        auto subExpressions = [](double w0, double w1, double w2,
                                 double &f1, double &f2, double &f3,
                                 double &g0, double &g1, double &g2)
        {
            const double temp0 = w0 + w1;
            const double temp1 = w0 * w0;
            const double temp2 = temp1 + w1 * temp0;
            f1 = temp0 + w2;
            f2 = temp2 + w2 * f1;
            f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
            g0 = f2 + w0 * (f1 + w0);
            g1 = f2 + w1 * (f1 + w1);
            g2 = f2 + w2 * (f1 + w2);
        };

        // iterate over all faces to compute integrals
        for (const auto &face : mesh.faces)
        {
            const auto &v0 = mesh.vertices[face[0]];
            const auto &v1 = mesh.vertices[face[1]];
            const auto &v2 = mesh.vertices[face[2]];

            const auto e1 = v1 - v0;
            const auto e2 = v2 - v0;
            const auto d = e1.cross(e2);

            double f1x, f2x, f3x, g0x, g1x, g2x;
            double f1y, f2y, f3y, g0y, g1y, g2y;
            double f1z, f2z, f3z, g0z, g1z, g2z;

            subExpressions(v0.x(), v1.x(), v2.x(), f1x, f2x, f3x, g0x, g1x, g2x);
            subExpressions(v0.y(), v1.y(), v2.y(), f1y, f2y, f3y, g0y, g1y, g2y);
            subExpressions(v0.z(), v1.z(), v2.z(), f1z, f2z, f3z, g0z, g1z, g2z);

            integrals[0] += d.x() * f1x;
            integrals[1] += d.x() * f2x;
            integrals[2] += d.y() * f2y;
            integrals[3] += d.z() * f2z;
            integrals[4] += d.x() * f3x;
            integrals[5] += d.y() * f3y;
            integrals[6] += d.z() * f3z;

            integrals[7] += d.x() * (v0.y() * g0x + v1.y() * g1x + v2.y() * g2x);
            integrals[8] += d.y() * (v0.z() * g0y + v1.z() * g1y + v2.z() * g2y);
            integrals[9] += d.z() * (v0.x() * g0z + v1.x() * g1z + v2.x() * g2z);
        }
        for (int i = 0; i < 10; i++)
            integrals[i] *= multipliers[i] * density;

        // calculate mass
        const double mass = integrals[0];
        if (mass <= std::numeric_limits<double>::epsilon())
        {
            massProps.inverseMass = 0.0;
            massProps.centerOfMass = Eigen::Vector3d::Zero();
            massProps.inverseInertiaBody = Eigen::Matrix3d::Zero();
            return;
        }
        massProps.inverseMass = 1.0 / mass;

        // calculate center of mass
        const auto centerOfMass = Eigen::Vector3d(
            integrals[1] / mass,
            integrals[2] / mass,
            integrals[3] / mass);
        massProps.centerOfMass = centerOfMass;

        // calculate inertia tensor
        Eigen::Matrix3d inertia;
        inertia(0, 0) = integrals[5] + integrals[6];
        inertia(1, 1) = integrals[4] + integrals[6];
        inertia(2, 2) = integrals[4] + integrals[5];

        inertia(0, 1) = inertia(1, 0) = -integrals[7];
        inertia(1, 2) = inertia(2, 1) = -integrals[8];
        inertia(0, 2) = inertia(2, 0) = -integrals[9];

        // move inertia to the center of mass
        const double centerOfMassSquaredNorm = centerOfMass.squaredNorm();
        const auto centerOfMassTransposed = centerOfMass.transpose();
        const auto parallelAxis = mass * (centerOfMassSquaredNorm * Eigen::Matrix3d::Identity() -
                                          centerOfMass * centerOfMassTransposed);
        const auto inertiaCenterOfMass = inertia - parallelAxis;

        const double det = inertiaCenterOfMass.determinant();
        if (std::abs(det) < DET_EPS)
            massProps.inverseInertiaBody = Eigen::Matrix3d::Zero();
        else
            massProps.inverseInertiaBody = inertiaCenterOfMass.inverse();
    }
    void SolveCollidingContacts(std::vector<Contact> &contacts)
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
    void SolveRestingContacts(const std::vector<Contact> &restingContacts, double t)
    {
        const size_t nContacts = restingContacts.size();

        auto aMat = ComputeAMatrix(restingContacts);
        auto bVec = ComputeBVector(restingContacts);

        auto fVec = SolveQuadratic(aMat, bVec);

        for (size_t i = 0; i < nContacts; i++)
        {
            auto &A = restingContacts[i].a;
            auto &B = restingContacts[i].b;
            auto fn = fVec[i] * restingContacts[i].normal;

            A.force += fn;
            A.torque += (restingContacts[i].position - A.state.position).cross(fn);

            B.force -= fn;
            B.torque -= (restingContacts[i].position - B.state.position).cross(fn);
        }
    }

    Eigen::VectorXd ComputeBVector(const std::vector<Contact> &contacts)
    {
        auto b = Eigen::VectorXd(contacts.size());
        for (size_t i = 0; i < contacts.size(); i++)
        {
            const Contact &c = contacts[i];
            const Rigidbody &A = c.a;
            const Rigidbody &B = c.b;

            const auto &n = c.normal;
            const auto &ra = c.position - A.state.position;
            const auto &rb = c.position - B.state.position;

            const auto &forceExtA = A.force;
            const auto &forceExtB = B.force;
            const auto &torqueExtA = A.torque;
            const auto &torqueExtB = B.torque;

            auto aExtPart = Eigen::Vector3d();
            auto aVelPart = Eigen::Vector3d();
            auto bExtPart = Eigen::Vector3d();
            auto bVelPart = Eigen::Vector3d();

            aExtPart = forceExtA * A.constants.inverseMass + ((A.derived.inverseInertia * torqueExtA).cross(ra));
            bExtPart = forceExtB * B.constants.inverseMass + ((B.derived.inverseInertia * torqueExtB).cross(rb));

            aVelPart = A.derived.angularVelocity.cross(A.derived.angularVelocity.cross(ra)) +
                       (A.derived.inverseInertia * A.state.angularMomentum.cross(A.derived.angularVelocity)).cross(ra);
            bVelPart = B.derived.angularVelocity.cross(B.derived.angularVelocity.cross(rb)) +
                       (B.derived.inverseInertia * B.state.angularMomentum.cross(B.derived.angularVelocity)).cross(rb);

            auto nDot = c.ComputeNDot();
            double k1 = n.dot((aExtPart + aVelPart) - (bExtPart + bVelPart));
            double k2 = 2.0 * nDot.dot(A.GetPointVelocity(c.position) - B.GetPointVelocity(c.position));

            b[i] = k1 + k2;
        }

        return b;
    }
    Eigen::MatrixXd ComputeAMatrix(const std::vector<Contact> &contacts)
    {
        auto a = Eigen::MatrixXd(contacts.size(), contacts.size());
        const size_t nContacts = contacts.size();
        for (size_t i = 0; i < nContacts; i++)
            for (size_t j = 0; j < nContacts; j++)
                a(i, j) = ComputeAAt(contacts[i], contacts[j]);

        return a;
    }
    double ComputeAAt(const Contact &ci, const Contact &cj)
    {
        if ((&ci.a != &cj.a) && (&ci.b != &cj.b) &&
            (&ci.a != &cj.b) && (&ci.b != &cj.a))
            return 0.0;

        const auto &A = ci.a;
        const auto &B = ci.b;

        const auto &ni = ci.normal;
        const auto &nj = cj.normal;
        const auto &pi = ci.position;
        const auto &pj = cj.position;
        const auto &ra = pi - A.state.position;
        const auto &rb = pi - B.state.position;

        auto forceOnA = Eigen::Vector3d{};
        auto torqueOnA = Eigen::Vector3d{};
        if (&cj.a == &ci.a)
        {
            forceOnA = nj;
            torqueOnA = (pj - A.state.position).cross(nj);
        }
        else if (&cj.b == &ci.a)
        {
            forceOnA = -nj;
            torqueOnA = (pj - A.state.position).cross(nj);
        }

        auto forceOnB = Eigen::Vector3d{};
        auto torqueOnB = Eigen::Vector3d{};
        if (&cj.a == &ci.b)
        {
            forceOnB = nj;
            torqueOnB = (pj - B.state.position).cross(nj);
        }
        else if (&cj.b == &ci.b)
        {
            forceOnB = -nj;
            torqueOnB = (pj - B.state.position).cross(nj);
        }

        auto aLinear = forceOnA * A.constants.inverseMass;
        auto aAngular = (A.derived.inverseInertia * torqueOnA).cross(ra);

        auto bLinear = forceOnB * B.constants.inverseMass;
        auto bAngular = (B.derived.inverseInertia * torqueOnB).cross(rb);

        return ni.dot((aLinear + aAngular) - (bLinear + bAngular));
    }

    Eigen::VectorXd SolveQuadratic(const Eigen::MatrixXd &A, const Eigen::VectorXd &b)
    {
        const int N = A.cols();

        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(N, N);
        Eigen::VectorXd c = Eigen::VectorXd::Zero(N);

        Eigen::MatrixXd CI(2 * N, N);
        Eigen::VectorXd ci0(2 * N);

        CI.topRows(N) = -A;
        ci0.head(N) = b;
        CI.bottomRows(N) = -Eigen::MatrixXd::Identity(N, N);
        ci0.tail(N) = Eigen::VectorXd::Zero(N);

        Eigen::MatrixXd CE(0, N);
        Eigen::VectorXd ce0(0);

        Eigen::VectorXd f(N);
        solve_quadprog(H, c, CE, ce0, CI, ci0, f);

        return f;
    }
}