#pragma once

#include "Mesh.hpp"

namespace Physik
{
    class TriangleMesh : public Mesh
    {
    public:
        TriangleMesh() = default;
        TriangleMesh(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<uint32_t, 3>> &faces);

    public:
        static TriangleMesh CreateCube(double width, double height, double depth);
    };
}