#include "Math/Math.hpp"
#include "Math/eiquadprog.hpp"

void ComputeContactForces(const std::vector<Contact> &restingContacts, double t)
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

        A.accumForce += fn;
        A.accumTorque += (restingContacts[i].position - A.state.position).cross(fn);

        B.accumForce -= fn;
        B.accumTorque -= (restingContacts[i].position - B.state.position).cross(fn);
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

        const auto &forceExtA = A.accumForce;
        const auto &forceExtB = B.accumForce;
        const auto &torqueExtA = A.accumTorque;
        const auto &torqueExtB = B.accumTorque;

        auto aExtPart = Eigen::Vector3d();
        auto aVelPart = Eigen::Vector3d();
        auto bExtPart = Eigen::Vector3d();
        auto bVelPart = Eigen::Vector3d();

        aExtPart = forceExtA / A.constants.mass + ((A.derived.thetaInv * torqueExtA).cross(ra));
        bExtPart = forceExtB / B.constants.mass + ((B.derived.thetaInv * torqueExtB).cross(rb));

        aVelPart = A.derived.angularVelocity.cross(A.derived.angularVelocity.cross(ra)) +
                   (A.derived.thetaInv * A.state.angularMomentum.cross(A.derived.angularVelocity)).cross(ra);
        bVelPart = B.derived.angularVelocity.cross(B.derived.angularVelocity.cross(rb)) +
                   (B.derived.thetaInv * B.state.angularMomentum.cross(B.derived.angularVelocity)).cross(rb);

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

    auto aLinear = forceOnA / A.constants.mass;
    auto aAngular = (A.derived.thetaInv * torqueOnA).cross(ra);

    auto bLinear = forceOnB / B.constants.mass;
    auto bAngular = (B.derived.thetaInv * torqueOnB).cross(rb);

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