#pragma once

#include <Eigen/Dense>

#include <array>
#include <cstdint>

namespace Physik
{
    struct TriangleMesh
    {
        static TriangleMesh CreateCube(double width, double height, double depth);

        std::vector<Eigen::Vector3d> vertices;
        std::vector<std::array<uint32_t, 3>> faces;
    };
}