#pragma once

#include <Eigen/Dense>

#include <vector>

namespace Physik
{
    class Mesh
    {
    public:
        Mesh(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::vector<uint32_t>> &faces)
            : vertices(vertices), faces(faces) {}

        virtual ~Mesh() = default;

        const std::vector<Eigen::Vector3d> &GetVertices() const { return vertices; }
        const std::vector<std::vector<uint32_t>> &GetFaces() const { return faces; }

    protected:
        std::vector<Eigen::Vector3d> vertices;
        std::vector<std::vector<uint32_t>> faces;
    };
}