#pragma once

#include <Eigen/Dense>

#include <vector>
#include <cstdint>

namespace Physik
{
    struct PolygonMesh
    {
        static PolygonMesh CreateCube(double width, double height, double depth);

        std::vector<Eigen::Vector3d> vertices;
        std::vector<std::vector<uint32_t>> faces;
    };
}