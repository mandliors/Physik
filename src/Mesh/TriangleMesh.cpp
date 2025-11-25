#include "TriangleMesh.hpp"

namespace Physik
{
    TriangleMesh TriangleMesh::CreateCube(double width, double height, double depth)
    {
        TriangleMesh cubeMesh;
        cubeMesh.vertices = {
            {-width / 2, -height / 2, -depth / 2},
            {width / 2, -height / 2, -depth / 2},
            {width / 2, height / 2, -depth / 2},
            {-width / 2, height / 2, -depth / 2},
            {-width / 2, -height / 2, depth / 2},
            {width / 2, -height / 2, depth / 2},
            {width / 2, height / 2, depth / 2},
            {-width / 2, height / 2, depth / 2},
        };
        cubeMesh.faces = {
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
        };
        return cubeMesh;
    }
}