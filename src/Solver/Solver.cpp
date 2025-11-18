#include "Solver.hpp"
#include "Math/eiquadprog.hpp"

void Solver::Solve(const std::vector<std::reference_wrapper<Rigidbody>> &bodies) const
{
	for (Rigidbody &body : bodies)
		body.ClearForces();

	for (size_t i = 0; i < bodies.size(); i++)
		for (size_t j = i + 1; j < bodies.size(); j++)
			bodies[i].get().CollideWith(bodies[j].get());

	for (Rigidbody &body : bodies)
		body.ApplyForces();
}

void Solver::FindAllCollisions(std::vector<Contact> &contacts) const
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
				OdeDiscontinuous();
			}
		}
	} while (collisionDetected);
}
void Solver::ComputeContactForces(const std::vector<Contact> &restingContacts, float t) const
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
		A.torque += (restingContacts[i].position - A.position).cross(fn);

		B.force -= fn;
		B.torque -= (restingContacts[i].position - B.position).cross(fn);
	}
}

Eigen::VectorXd Solver::ComputeBVector(const std::vector<Contact> &contacts) const
{
	auto b = Eigen::VectorXd(contacts.size());
	for (size_t i = 0; i < contacts.size(); i++)
	{
		const Contact &c = contacts[i];
		const Rigidbody &A = c.a;
		const Rigidbody &B = c.b;

		const auto &n = c.normal;
		const auto &ra = c.position - A.position;
		const auto &rb = c.position - B.position;

		const auto &forceExtA = A.force;
		const auto &forceExtB = B.force;
		const auto &torqueExtA = A.torque;
		const auto &torqueExtB = B.torque;

		auto aExtPart = Eigen::Vector3d();
		auto aVelPart = Eigen::Vector3d();
		auto bExtPart = Eigen::Vector3d();
		auto bVelPart = Eigen::Vector3d();

		aExtPart = forceExtA / A.mass + ((A.thetaInv * torqueExtA).cross(ra));
		bExtPart = forceExtB / B.mass + ((B.thetaInv * torqueExtB).cross(rb));

		aVelPart = A.angularVelocity.cross(A.angularVelocity.cross(ra)) +
				   (A.thetaInv * A.angularMomentum.cross(A.angularVelocity)).cross(ra);
		bVelPart = B.angularVelocity.cross(B.angularVelocity.cross(rb)) +
				   (B.thetaInv * B.angularMomentum.cross(B.angularVelocity)).cross(rb);

		auto nDot = c.ComputeNDot();
		double k1 = n.dot((aExtPart + aVelPart) - (bExtPart + bVelPart));
		double k2 = 2.0 * nDot.dot(A.GetPointVelocity(c.position) - B.GetPointVelocity(c.position));

		b[i] = k1 + k2;
	}

	return b;
}

Eigen::MatrixXd Solver::ComputeAMatrix(const std::vector<Contact> &contacts) const
{
	auto a = Eigen::MatrixXd(contacts.size(), contacts.size());
	const size_t nContacts = contacts.size();
	for (size_t i = 0; i < nContacts; i++)
		for (size_t j = 0; j < nContacts; j++)
			a(i, j) = ComputeAAt(contacts[i], contacts[j]);

	return a;
}
double Solver::ComputeAAt(const Contact &ci, const Contact &cj) const
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
	const auto &ra = pi - A.position;
	const auto &rb = pi - B.position;

	auto forceOnA = Eigen::Vector3d{};
	auto torqueOnA = Eigen::Vector3d{};
	if (&cj.a == &ci.a)
	{
		forceOnA = nj;
		torqueOnA = (pj - A.position).cross(nj);
	}
	else if (&cj.b == &ci.a)
	{
		forceOnA = -nj;
		torqueOnA = (pj - A.position).cross(nj);
	}

	auto forceOnB = Eigen::Vector3d{};
	auto torqueOnB = Eigen::Vector3d{};
	if (&cj.a == &ci.b)
	{
		forceOnB = nj;
		torqueOnB = (pj - B.position).cross(nj);
	}
	else if (&cj.b == &ci.b)
	{
		forceOnB = -nj;
		torqueOnB = (pj - B.position).cross(nj);
	}

	auto aLinear = forceOnA / A.mass;
	auto aAngular = (A.thetaInv * torqueOnA).cross(ra);

	auto bLinear = forceOnB / B.mass;
	auto bAngular = (B.thetaInv * torqueOnB).cross(rb);

	return ni.dot((aLinear + aAngular) - (bLinear + bAngular));
}

Eigen::VectorXd Solver::SolveQuadratic(const Eigen::MatrixXd &A, const Eigen::VectorXd &b) const
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

void Solver::OdeDiscontinuous() const {}