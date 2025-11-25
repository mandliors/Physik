#include "TriangleMesh.hpp"

namespace Physik
{
    TriangleMesh::TriangleMesh(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<uint32_t, 3>> &faces)
        : Mesh(vertices, {})
    {
        this->faces.resize(faces.size());
        for (size_t i = 0; i < faces.size(); i++)
            this->faces[i] = {faces[i][0], faces[i][1], faces[i][2]};
    }

    TriangleMesh TriangleMesh::CreateCube(double width, double height, double depth)
    {
        return TriangleMesh{
            {
                {-width / 2, -height / 2, -depth / 2},
                {width / 2, -height / 2, -depth / 2},
                {width / 2, height / 2, -depth / 2},
                {-width / 2, height / 2, -depth / 2},
                {-width / 2, -height / 2, depth / 2},
                {width / 2, -height / 2, depth / 2},
                {width / 2, height / 2, depth / 2},
                {-width / 2, height / 2, depth / 2},
            },
            {
                {0, 2, 1},
                {0, 3, 2},
                {4, 5, 6},
                {4, 6, 7},
                {0, 1, 5},
                {0, 5, 4},
                {2, 3, 7},
                {2, 7, 6},
                {1, 2, 6},
                {1, 6, 5},
                {0, 7, 3},
                {0, 4, 7},
            },
        };
    }
}