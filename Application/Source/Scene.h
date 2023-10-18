#pragma once

#include <GDT/Vector3f.h>
#include <GDT/OpenGL.h>

#include <vector>

#include "Mesh.h"

#include "ShapeOp/Solver.h"

class GLObject
{
public:
    void setData(const std::vector<Vector3f>& vertices, const std::vector<Vector3f>& normals, const std::vector<int>& indices)
    {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3f), vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);

        glGenBuffers(1, &nbo);
        glBindBuffer(GL_ARRAY_BUFFER, nbo);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Vector3f), normals.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(1);

        glGenBuffers(1, &ibo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int), indices.data(), GL_STATIC_DRAW);
    }

    void setFloorData()
    {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        std::vector<Vector3f> vertices
        {
            Vector3f(-1, 0, 1),
            Vector3f(1, 0, 1),
            Vector3f(-1, 0, -1),
            Vector3f(1, 0, -1)
        };

        std::vector<Vector3f> normals
        {
            Vector3f(0, 1, 0),
            Vector3f(0, 1, 0),
            Vector3f(0, 1, 0),
            Vector3f(0, 1, 0)
        };

        std::vector<int> indices
        {
            0, 1, 2, 2, 1, 3
        };

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3f), vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);

        glGenBuffers(1, &nbo);
        glBindBuffer(GL_ARRAY_BUFFER, nbo);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Vector3f), normals.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(1);

        glGenBuffers(1, &ibo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int), indices.data(), GL_STATIC_DRAW);
    }

    void initializeColDebug(int numVerts)
    {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        std::vector<Vector3f> vertices(numVerts);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3f), vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);
    }

    GLuint vao;
    GLuint vbo;
    GLuint nbo;
    GLuint ibo;
};

class Scene
{
public:
    void storePositions()
    {
        int numVertices = 0;
        for (const Mesh& cell : cells)
        {
            numVertices += cell.vertices.size();
        }

        std::vector<Vector3f> linearPositions(numVertices);
        int i = 0;
        for (const Mesh& cell : cells)
        {
            for (int v = 0; v < cell.vertices.size(); v++)
            {
                linearPositions[i++] = cell.vertices[v];
            }
        }

        int previousSize = positions.cols();

        if (positions.cols() > 0)
            positions.conservativeResize(3, linearPositions.size());
        else
            positions.resize(3, linearPositions.size());

        for (int j = 0; j < linearPositions.size(); j++)
        {
            positions(0, j) = linearPositions[j].x;
            positions(1, j) = linearPositions[j].y;
            positions(2, j) = linearPositions[j].z;
        }
    }

public:
    GLObject obj;
    GLObject floor;
    GLObject colDebug;

    std::vector<Mesh> cells;

    ShapeOp::Matrix3X positions;
    ShapeOp::Matrix3X velocities;
};
