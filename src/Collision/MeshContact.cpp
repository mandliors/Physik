#include "MeshContact.hpp"

namespace Physik {
	MeshContact::MeshContact(const BaseCollider* a, const BaseCollider* b, const Eigen::Vector3d& point, const Eigen::Vector3d& normal)
		:a(a), b(b), point(point), normal(normal.normalized())
	{

	}
}