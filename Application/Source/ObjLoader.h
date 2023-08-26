#pragma once

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <string>

#include "ShapeOp/Solver.h"
#include "ShapeOp/Constraint.h"

struct Face
{
    int i0, i1, i2;
};

class Mesh
{
public:
    std::vector<Vector3f> vertices;
    std::vector<Vector3f> prevVertices;
    std::vector<Vector3f> normals;
    //std::vector<int> indices;
    std::vector<Face> faces;
    std::vector<Vector3f> faceNormals;
    std::vector<int> volumeConstraintIds;
    std::vector<int> collisionConstraintIds;
};

bool loadObject(std::string filePath, Mesh& mesh)
{
    Assimp::Importer importer;

    unsigned int flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType;
    const aiScene* aiScene = importer.ReadFile(filePath, flags);

    if (!aiScene)
    {
        std::cout << "Failed to load model: " << importer.GetErrorString() << std::endl;
        return false;
    }

    const int numMeshes = aiScene->mNumMeshes;

    for (int i = 0; i < numMeshes; i++)
    {
        const aiMesh* aiMesh = aiScene->mMeshes[i];

        const int numVertices = aiMesh->mNumVertices;
        const int numFaces = aiMesh->mNumFaces;

        mesh.vertices.resize(numVertices + 1);
        mesh.prevVertices.resize(numVertices + 1);
        mesh.normals.resize(numVertices + 1);
        mesh.faces.resize(numFaces);

        std::cout << numVertices << " " << aiMesh->mNumFaces << std::endl;

        for (int j = 0; j < numVertices; j++)
        {
            mesh.vertices[j] = Vector3f(aiMesh->mVertices[j].x, aiMesh->mVertices[j].y, aiMesh->mVertices[j].z);
            mesh.prevVertices[j] = Vector3f(aiMesh->mVertices[j].x, aiMesh->mVertices[j].y, aiMesh->mVertices[j].z);
            mesh.normals[j] = mesh.vertices[j]; // Vector3f(aiMesh->mNormals[j].x, aiMesh->mNormals[j].y, aiMesh->mNormals[j].z);
        }
        // Set last vertex in the middle to allow making tetrahedra
        mesh.vertices[numVertices] = Vector3f(0, 0, 0);
        mesh.normals[numVertices] = Vector3f(0, 0, 0);

        for (int j = 0; j < aiMesh->mNumFaces; j++)
        {
            const aiFace& aiFace = aiMesh->mFaces[j];
            if (aiFace.mNumIndices < 3)
                continue;
            mesh.faces[j] = Face{ (int) aiFace.mIndices[0], (int)aiFace.mIndices[1], (int)aiFace.mIndices[2] };
        }
    }
}

void storeVertices(const Mesh& mesh, std::vector<Vector3f>& vertices)
{
    vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
}

void storeNormals(const Mesh& mesh, std::vector<Vector3f>& normals)
{
    normals.insert(normals.end(), mesh.normals.begin(), mesh.normals.end());
}

void storeIndices(const Mesh& mesh, std::vector<int>& indices, int offset)
{
    // Store edges in constraints
    for (int j = 0; j < mesh.faces.size(); j++)
    {
        const Face& face = mesh.faces[j];

        std::cout << "Face: " << face.i0 << " " << face.i1 << " " << face.i2 << std::endl;

        indices.push_back(offset + face.i0);
        indices.push_back(offset + face.i1);
        indices.push_back(offset + face.i1);
        indices.push_back(offset + face.i2);
        indices.push_back(offset + face.i2);
        indices.push_back(offset + face.i0);

        indices.push_back(offset + face.i0);
        indices.push_back(offset + mesh.vertices.size() - 1);
        indices.push_back(offset + face.i1);
        indices.push_back(offset + mesh.vertices.size() - 1);
        indices.push_back(offset + face.i2);
        indices.push_back(offset + mesh.vertices.size() - 1);
    }
}

void verticesToSimulation(const std::vector<Vector3f>& vertices, ShapeOp::Matrix3X& p, ShapeOp::Solver& s)
{
    int previousSize = p.cols();

    if (p.cols() > 0)
        p.conservativeResize(3, p.cols() + vertices.size());
    else
        p.resize(3, vertices.size());

    for (int j = 0; j < vertices.size(); j++)
    {
        p(0, previousSize + j) = vertices[j].x;
        p(1, previousSize + j) = vertices[j].y;
        p(2, previousSize + j) = vertices[j].z;
    }
}

void makeCellConstraint(Mesh& mesh, ShapeOp::Solver& s, int offset)
{
    // Store edges in constraints
    mesh.volumeConstraintIds.resize(mesh.faces.size());
    for (int j = 0; j < mesh.faces.size(); j++)
    {
        const Face& face = mesh.faces[j];

        std::cout << "Face: " << face.i0 << " " << face.i1 << " " << face.i2 << std::endl;

        {
            std::vector<int> id_vector = { offset + face.i0, offset + face.i1, offset + face.i2, offset + (int) mesh.vertices.size() - 1 };
            auto c = std::make_shared<ShapeOp::VolumeConstraint>(id_vector, 100, s.getPoints(), 0.95, 1.05);
            int volConstraintId = s.addConstraint(c);
            
            mesh.volumeConstraintIds[j] = volConstraintId;

            auto sc = std::make_shared<ShapeOp::TetrahedronStrainConstraint>(id_vector, 200, s.getPoints(), 0.95, 1.05);
            s.addConstraint(sc);

            //c->setRangeMin(0.95);
            //c->setRangeMax(1.05);
        }
    }

    //if (offset == 0)
    //{
    //    mesh.collisionConstraintIds.resize(mesh.vertices.size());
    //    for (int i = 0; i < mesh.vertices.size(); i++)
    //    {
    //        std::vector<int> id_vector = { offset + i };
    //        auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, 10, s.getPoints());
    //        int colConstraintId = s.addConstraint(c);

    //        mesh.collisionConstraintIds[i] = colConstraintId;
    //    }
    //}
}

void computeNormals(Mesh& mesh)
{
    mesh.faceNormals.resize(mesh.faces.size());

    for (int i = 0; i < mesh.faces.size(); i++)
    {
        const Face& face = mesh.faces[i];

        Vector3f v1 = mesh.vertices[face.i1] - mesh.vertices[face.i0];
        Vector3f v2 = mesh.vertices[face.i2] - mesh.vertices[face.i0];

        mesh.faceNormals[i] = normalize(cross(v1, v2));

        //std::cout << mesh.vertices[face.i0] << mesh.vertices[face.i1] << mesh.vertices[face.i2] << std::endl;
        //std::cout << mesh.faceNormals[i] << std::endl;
    }
}

void computeBarycentricCoordinates(Vector3f p, Vector3f a, Vector3f b, Vector3f c, float& u, float& v, float& w)
{
    Vector3f v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(v2, v0);
    float d21 = dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

Vector3f findClosestPointOnTriangle(Vector3f p, const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& N)
{
    // Calculate some vector from a triangle vertex to P
    Vector3f vertexToP = p - v0;
    // Subtract the portion of the vector in the direction of the triangle normal
    Vector3f projP = v0 + vertexToP - N * dot(N, vertexToP);

    // Find the barycentric coordinates of the projected point
    float u, v, w;
    computeBarycentricCoordinates(p, v0, v1, v2, u, v, w);

    // Check if the projected point lies within the triangle
    if (u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f && w >= 0.0f && w <= 1.0f)
    {
        // The projected point lies inside the triangle, then this is the closest point
        return v0 * u + v1 * v + v2 * w;
    }
    else
    {
        // The projected point lies outside the triangle, find out if the closest point is on an edge or a vertex
        float minSqrDist = 100;
        Vector3f closestPoint(NAN, NAN, NAN);

        // Project the projected point on each of the triangles edges, and find the minimum distance
        Vector3f edges[] = { v1 - v0, v2 - v1, v0 - v2 };
        for (int i = 0; i < 3; i++)
        {
            float w = dot(projP - v0, edges[i]) / dot(edges[i], edges[i]);

            // Check if the projected point lies on the triangle edge
            if (w >= 0.0f && w <= 1.0f)
            {
                Vector3f edgeProjP = v0 + edges[i] * w;
                float sqrDist = (p - edgeProjP).sqrMagnitude();
                if (sqrDist < minSqrDist)
                {
                    minSqrDist = sqrDist;
                    closestPoint = edgeProjP;
                }
            }
        }

        // Find the distance of the projected point to each of the triangle vertex, and find the minimum distance
        Vector3f vertices[] = { v0, v1, v2 };
        for (int i = 0; i < 3; i++)
        {
            float sqrDist = (p - vertices[i]).sqrMagnitude();
            if (sqrDist < minSqrDist)
            {
                minSqrDist = sqrDist;
                closestPoint = vertices[i];
            }
        }

        return closestPoint;
    }
}

Vector3f findClosestPointOnMesh(Vector3f p, const Mesh& mesh)
{
    float minSqrDist = 100;
    Vector3f closestPoint;
    for (int i = 0; i < mesh.faces.size(); i++)
    {
        const Face& face = mesh.faces[i];

        Vector3f v0 = mesh.vertices[face.i0];
        Vector3f v1 = mesh.vertices[face.i1];
        Vector3f v2 = mesh.vertices[face.i2];

        Vector3f cp = findClosestPointOnTriangle(p, v0, v1, v2, mesh.faceNormals[i]);

        float sqrDist = (cp - p).sqrMagnitude();

        if (sqrDist < minSqrDist)
        {
            minSqrDist = sqrDist;
            closestPoint = cp;
        }
    }

    return closestPoint;
}

Vector3f findClosestVertex(Vector3f p, const Mesh& mesh)
{
    float minDist = 100;
    int closestVertex = -1;

    for (int i = 0; i < mesh.vertices.size(); i++)
    {
        const Vector3f& v = mesh.vertices[i];
        float dist = (p - v).length();
        if (dist < minDist)
        {
            minDist = dist;
            closestVertex = i;
        }
    }
    return mesh.vertices[closestVertex];
}

bool triangleIntersect(Vector3f o, Vector3f d, const Mesh& mesh, const Face& face, float& t)
{
    float epsilon = 0.000001f;

    const Vector3f& v0 = mesh.vertices[face.i0];
    const Vector3f& v1 = mesh.vertices[face.i1];
    const Vector3f& v2 = mesh.vertices[face.i2];

    // Find the edge vectors
    Vector3f e1 = v1 - v0;
    Vector3f e2 = v2 - v0;

    // Calculate determinant
    Vector3f P = cross(d, e2);
    float det = dot(e1, P);

    // If det near zero, ray lies in plane of triangle
    if (det > -epsilon && det < epsilon) {
        return false;
    }
    float invDet = 1.0f / det;

    // Calculate distance from v0 to ray origin
    Vector3f T = o - v0;

    // Calculate the u parameter and test bound
    float a = dot(T, P) * invDet;
    if (a < 0 || a > 1) {
        // The intersection lies outside of the triangle
        return false;
    }

    // Calculate the v parameter and test bound
    Vector3f Q = cross(T, e1);

    float b = dot(d, Q) * invDet;
    if (b < 0 || a + b > 1) {
        // The intersection lies outside of the triangle
        return false;
    }

    t = dot(e2, Q) * invDet;
    //return Vector3f(t, a, b);
    return true;
}

bool vertexInCell(Vector3f v, const Mesh& mesh, float& t)
{
    float minT = 100;
    int fHit = -1;

    for (int f = 0; f < mesh.faces.size(); f++)
    {
        float t = 100;
        bool hit = triangleIntersect(v, Vector3f(0, 1, 0), mesh, mesh.faces[f], t);
        if (hit && t < minT && t > 0)
        {
            minT = t;
            fHit = f;
        }
    }
    t = minT;
    // A triangle was hit, determine if we are inside or outside
    if (fHit != -1)
    {
        Vector3f faceNormal = mesh.faceNormals[fHit];
        Vector3f v0 = mesh.vertices[mesh.faces[fHit].i0];
        Vector3f faceToVertex = normalize(v - v0);

        float d = dot(faceNormal, faceToVertex);
        if (d < 0)
        {
            // Point is inside
            return true;
        }
    }
    return false;
}

//bool loadObjects()
//{
//    Assimp::Importer importer;

//    unsigned int flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType;
//    const aiScene* aiScene = importer.ReadFile("Resources/IcosphereHigh.obj", flags);

//    if (!aiScene)
//    {
//        std::cout << "Failed to load model: " << importer.GetErrorString() << std::endl;
//        return false;
//    }

//    for (int i = 0; i < aiScene->mNumMeshes; i++)
//    {
//        const aiMesh* aiMesh = aiScene->mMeshes[i];
//        std::cout << "Num vertices: " << aiMesh->mNumVertices << std::endl;
//        std::vector<Vector3f> vertices(aiMesh->mNumVertices + 1);
//        normals.resize(aiMesh->mNumVertices + 1);

//        std::vector<Face> faces(aiMesh->mNumFaces);

//        std::cout << aiMesh->mNumVertices << " " << aiMesh->mNumFaces << std::endl;

//        for (int j = 0; j < aiMesh->mNumVertices; j++)
//        {
//            vertices[j] = Vector3f(aiMesh->mVertices[j].x, aiMesh->mVertices[j].y, aiMesh->mVertices[j].z);
//            normals[j] = vertices[j]; // Vector3f(aiMesh->mNormals[j].x, aiMesh->mNormals[j].y, aiMesh->mNormals[j].z);
//        }
//        vertices[aiMesh->mNumVertices] = Vector3f(0, 0, 0);
//        normals[aiMesh->mNumVertices] = Vector3f(0, 0, 0);

//        for (int j = 0; j < aiMesh->mNumFaces; j++)
//        {
//            const aiFace& aiFace = aiMesh->mFaces[j];
//            if (aiFace.mNumIndices < 3)
//                continue;
//            faces[j] = Face{ aiFace.mIndices[0], aiFace.mIndices[1], aiFace.mIndices[2] };
//        }

//        p.resize(3, vertices.size());
//        for (int j = 0; j < vertices.size(); j++)
//        {
//            p(0, j) = vertices[j].x;
//            p(1, j) = vertices[j].y;
//            p(2, j) = vertices[j].z;
//        }
//        std::cout << "Beep" << std::endl;
//        s.setPoints(p);

//        simulation.state.q.resize(vertices.size() * 3, 1);
//        std::cout << simulation.state.q << std::endl;
//        // Store vertices in q
//        for (int j = 0; j < vertices.size(); j++)
//        {
//            simulation.state.q(j * 3 + 0) = vertices[j].x;
//            simulation.state.q(j * 3 + 1) = vertices[j].y;
//            simulation.state.q(j * 3 + 2) = vertices[j].z;
//        }

//        std::cout << "AFTER" << std::endl;
//        std::cout << simulation.state.q << std::endl;
//        std::cout << "AFTER2" << std::endl;

//        volumeConstraintIds.resize(faces.size());

//        // Store edges in constraints
//        for (int j = 0; j < faces.size(); j++)
//        {
//            Face& face = faces[j];
//            //simulation.constraints.push_back(new SpringConstraint((int)face.i0, (int)face.i1, 0.3f));
//            //simulation.constraints.push_back(new SpringConstraint((int)face.i1, (int)face.i2, 0.3f));
//            //simulation.constraints.push_back(new SpringConstraint((int)face.i2, (int)face.i0, 0.3f));
//            std::cout << "Face: " << face.i0 << " " << face.i1 << " " << face.i2 << std::endl;
//            simulation.constraints.push_back(new TetConstraint(face.i0, face.i1, face.i2, aiMesh->mNumVertices, simulation.state.q));

//            {
//                std::vector<int> id_vector;
//                id_vector.push_back(face.i0); id_vector.push_back(face.i1); id_vector.push_back(face.i2); id_vector.push_back(aiMesh->mNumVertices);
//                auto c = std::make_shared<ShapeOp::VolumeConstraint>(id_vector, 500, s.getPoints(), 0.95, 1.05);
//                int volConstraintId = s.addConstraint(c);

//                volumeConstraintIds[j] = volConstraintId;

//                auto sc = std::make_shared<ShapeOp::TetrahedronStrainConstraint>(id_vector, 2000, s.getPoints(), 0.95, 1.05);
//                s.addConstraint(sc);

//                //c->setRangeMin(0.95);
//                //c->setRangeMax(1.05);
//            }

//            indices.push_back(face.i0);
//            indices.push_back(face.i1);
//            indices.push_back(face.i1);
//            indices.push_back(face.i2);
//            indices.push_back(face.i2);
//            indices.push_back(face.i0);

//            indices.push_back(face.i0);
//            indices.push_back(aiMesh->mNumVertices);
//            indices.push_back(face.i1);
//            indices.push_back(aiMesh->mNumVertices);
//            indices.push_back(face.i2);
//            indices.push_back(aiMesh->mNumVertices);
//        }
//    }
//}
