#pragma once

#include <Eigen/Dense>

#include <vector>

namespace Physik
{
    struct Mesh
    {
        static Mesh CreateCube(double width, double height, double depth);

        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> faces;
    };
}