#pragma once

#include "Mesh.hpp"

namespace Physik
{
    class PolygonMesh : public Mesh
    {
    public:
        PolygonMesh() = default;
        PolygonMesh(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::vector<uint32_t>> &faces)
            : Mesh(vertices, faces) {}

    public:
        static PolygonMesh CreateCube(double width, double height, double depth);
    };
}