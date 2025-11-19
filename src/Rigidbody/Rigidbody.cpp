#include "Rigidbody/Rigidbody.hpp"

static Eigen::Quaterniond operator*(double v, const Eigen::Quaterniond &q);

Rigidbody::Rigidbody()
	: constants{
		  1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()},
	  state{Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)}, derived{Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)}, accumForce(3), accumTorque(3)
{
}

void Rigidbody::CalculateDerivedQuantities()
{
	derived.thetaInv = derived.rotationMat * constants.thetaBodyInv * derived.rotationMat.transpose();
	derived.rotationMat = state.orientation.toRotationMatrix();
	derived.velocity = state.linearMomentum / static_cast<float>(constants.mass);
	derived.angularVelocity = derived.thetaInv * state.angularMomentum;
}

Rigidbody::StateDot Rigidbody::CalculateStateDot() const
{
	return StateDot{
		derived.velocity,
		0.5 * Eigen::Quaterniond(0.0, derived.angularVelocity.x(), derived.angularVelocity.y(), derived.angularVelocity.z()) * state.orientation,
		force,
		torque};
}

void Rigidbody::Step(const OdeSolver &solver, double deltaTime)
{
	State endState;

	double *y0 = reinterpret_cast<double *>(&state);
	double *yend = reinterpret_cast<double *>(&endState);

	solver.Solve(y0, yend, sizeof(State) / sizeof(double), 0.0, deltaTime, deltaTime, [this](double t, double *y, double *ydot)
				 {
							// calculate forces at time t
							StateDot stateDot = CalculateStateDot();
							std::memcpy(ydot, reinterpret_cast<double *>(&stateDot), sizeof(StateDot)); });

	state = endState;
}

void Rigidbody::ClearForces()
{
	force = Eigen::Vector3d(3);
	torque = Eigen::Vector3d(3);
}

Eigen::Vector3d Rigidbody::GetPointVelocity(const Eigen::Vector3d &point) const
{
	return derived.velocity + derived.angularVelocity.cross(point - state.position);
}

static Eigen::Quaterniond operator*(double v, const Eigen::Quaterniond &q)
{
	return Eigen::Quaterniond(q.w() * v, q.x() * v, q.y() * v, q.z() * v);
}