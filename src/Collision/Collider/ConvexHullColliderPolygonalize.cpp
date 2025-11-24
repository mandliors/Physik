#define _CRT_SECURE_NO_WARNINGS

#include "ConvexHullCollider.hpp"

#include <map>
#include <stdlib.h>

class Edge {
public:
	int edgeIndices[2];	//in ascending order
	Edge(int e0, int e1)
	{
		if (e0 > e1)
		{
			edgeIndices[0] = e1;
			edgeIndices[1] = e0;
		}
		else
		{
			edgeIndices[0] = e0;
			edgeIndices[1] = e1;
		}
	}

	bool operator==(const Edge& other) const
	{
		int result = 0;
		result |= edgeIndices[0] - other.edgeIndices[0];
		result |= edgeIndices[1] - other.edgeIndices[1];
		return (bool)result;
	}

	static int comparator(const void* e0, const void* e1)
	{
		const Edge* a = static_cast<const Edge*>(e0);
		const Edge* b = static_cast<const Edge*>(e1);

		int temp = a->edgeIndices[0] - b->edgeIndices[0];
		if (!temp)
			return a->edgeIndices[1] - b->edgeIndices[1];
		return temp;
	}
};

class Face {
public:
	int faceIndices[3];
	Face(int f0, int f1, int f2)
	{
		faceIndices[0] = f0;
		faceIndices[1] = f1;
		faceIndices[2] = f2;
	}

	bool operator==(const Face& other) const
	{
		int result = 0;
		result |= faceIndices[0] - other.faceIndices[0];
		result |= faceIndices[1] - other.faceIndices[1];
		result |= faceIndices[2] - other.faceIndices[2];
		return (bool)result;
	}

	static int comparator(const void* f0, const void* f1)
	{
		const Face* a = static_cast<const Face*>(f0);
		const Face* b = static_cast<const Face*>(f1);

		int temp = a->faceIndices[0] - b->faceIndices[0];
		if (temp)
			return temp;

		temp = a->faceIndices[1] - b->faceIndices[1];
		if (temp)
			return temp;

		return a->faceIndices[2] - b->faceIndices[2];
	}
};

namespace Physik {

	void ConvexHullCollider::MergeDuplicates()	//the duplicate vertices are actually left among the vertices, but the edges don't use them anymore
	{
		struct VertexIndexMap {
			Eigen::Vector3d vertex;
			std::vector<long> indices;
			VertexIndexMap() :vertex(Eigen::Vector3d(0, 0, 0)), indices(std::vector<long>()) {}
		};
		static constexpr double MIN_SQR_DIFFERENCE = 0.0000001;
		std::vector<VertexIndexMap> sus;

		//calculate initial helpers
		for (int i = 0; i < vertices.size(); i++)
		{
			sus.push_back(VertexIndexMap());
			sus[i].vertex = vertices[i];
			sus[i].indices.push_back(i);
		}

		//merge the vertices
		for (int i = 0; i < sus.size() - 1; i++)
		{
			for (int j = i + 1; j < sus.size(); j++)
			{
				if ((sus[i].vertex - sus[j].vertex).squaredNorm() < MIN_SQR_DIFFERENCE)
				{
					sus[i].indices.push_back(sus[j].indices[0]);
					sus.erase(sus.cbegin() + j);
					j--;
				}
			}
		}

		//switch the indices
		std::map<long, Eigen::Vector3d*> prevIndexToVertex;
		std::map<Eigen::Vector3d*, long> newVertexToIndex;

		for (int i = 0; i < sus.size(); i++)
		{
			Eigen::Vector3d* temp = &(vertices[sus[i].indices[0]]);
			newVertexToIndex[temp] = sus[i].indices[0];
			for (long j : sus[i].indices)
				prevIndexToVertex[j] = temp;
		}

		for (long& index : triangles)
			index = newVertexToIndex[prevIndexToVertex[index]];
	}

	void ConvexHullCollider::Polygonalize()
	{
		std::vector<Edge> edges;
		std::vector<Face> faces;
		std::map<Edge*, std::vector<Face*>> edgeFaceMapper;

		//construct helper vectors
		for (int i = 0; i < triangles.size(); i += 3)
		{
			faces.push_back(Face(triangles[i], triangles[i + 1], triangles[i + 2]));
			edges.push_back(Edge(triangles[i], triangles[i + 1]));
			edges.push_back(Edge(triangles[i + 1], triangles[i + 2]));
			edges.push_back(Edge(triangles[i + 2], triangles[i]));
		}

		qsort(edges.data(), edges.size(), sizeof(Edge), Edge::comparator);
		qsort(faces.data(), faces.size(), sizeof(Face), Face::comparator);

		for (int i = edges.size() - 1; i >= 1; i--)
			if (edges[i] == edges[i - 1])
				edges.erase(edges.cbegin() + i);

		for (int i = faces.size() - 1; i >= 1; i--)
			if (faces[i] == faces[i - 1])
				faces.erase(faces.cbegin() + i);

		//construct helper map
		for (int i = 0; i < triangles.size(); i += 3)
		{
			Face face(triangles[i], triangles[i + 1], triangles[i + 2]);
			Edge edge0(triangles[i], triangles[i + 1]);
			Edge edge1(triangles[i + 1], triangles[i + 2]);
			Edge edge2(triangles[i + 2], triangles[i]);

			Face* pFace = static_cast<Face*>(bsearch(&face, faces.data(), faces.size(), sizeof(Face), Face::comparator));
			Edge* pEdge0 = static_cast<Edge*>(bsearch(&edge0, edges.data(), edges.size(), sizeof(Edge), Edge::comparator));
			Edge* pEdge1 = static_cast<Edge*>(bsearch(&edge1, edges.data(), edges.size(), sizeof(Edge), Edge::comparator));
			Edge* pEdge2 = static_cast<Edge*>(bsearch(&edge2, edges.data(), edges.size(), sizeof(Edge), Edge::comparator));



			if (edgeFaceMapper.contains(pEdge0))
				edgeFaceMapper[pEdge0].push_back(pFace);
			else
				edgeFaceMapper[pEdge0] = std::vector<Face*>{ pFace };

			if (edgeFaceMapper.contains(pEdge1))
				edgeFaceMapper[pEdge1].push_back(pFace);
			else
				edgeFaceMapper[pEdge1] = std::vector<Face*>{ pFace };

			if (edgeFaceMapper.contains(pEdge2))
				edgeFaceMapper[pEdge2].push_back(pFace);
			else
				edgeFaceMapper[pEdge2] = std::vector<Face*>{ pFace };
		}

		//NOTE 4 L8R: no need for convexness check when merging triangles, because a convex hull cannot have concave polygons anyway
	}

}