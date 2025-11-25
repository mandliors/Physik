#include "PolygonMesh.hpp"

namespace Physik
{
    PolygonMesh PolygonMesh::CreateCube(double width, double height, double depth)
    {
        PolygonMesh cubeMesh;
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
            {0, 3, 2, 1},
            {4, 5, 6, 7},
            {0, 1, 5, 4},
            {2, 3, 7, 6},
            {1, 2, 6, 5},
            {0, 4, 7, 3}};
        return cubeMesh;
    }
}