#include "Mesh.hpp"

namespace Physik
{
    Mesh Mesh::CreateCube(double width, double height, double depth)
    {
        Mesh cubeMesh;
        cubeMesh.vertices = {
            Eigen::Vector3d(-width / 2, -height / 2, -depth / 2),
            Eigen::Vector3d(width / 2, -height / 2, -depth / 2),
            Eigen::Vector3d(width / 2, height / 2, -depth / 2),
            Eigen::Vector3d(-width / 2, height / 2, -depth / 2),
            Eigen::Vector3d(-width / 2, -height / 2, depth / 2),
            Eigen::Vector3d(width / 2, -height / 2, depth / 2),
            Eigen::Vector3d(width / 2, height / 2, depth / 2),
            Eigen::Vector3d(-width / 2, height / 2, depth / 2),
        };
        cubeMesh.faces = {
            Eigen::Vector3i(0, 2, 1),
            Eigen::Vector3i(0, 3, 2),
            Eigen::Vector3i(4, 5, 6),
            Eigen::Vector3i(4, 6, 7),
            Eigen::Vector3i(0, 1, 5),
            Eigen::Vector3i(0, 5, 4),
            Eigen::Vector3i(2, 3, 7),
            Eigen::Vector3i(2, 7, 6),
            Eigen::Vector3i(1, 2, 6),
            Eigen::Vector3i(1, 6, 5),
            Eigen::Vector3i(0, 7, 3),
            Eigen::Vector3i(0, 4, 7),
        };
        return cubeMesh;
    }
}