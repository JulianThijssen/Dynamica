#pragma once

#include <GDT/Vector3f.h>

#include <vector>

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
