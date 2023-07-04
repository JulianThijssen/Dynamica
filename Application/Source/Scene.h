#pragma once

#include <GDT/Vector3f.h>
#include <GDT/OpenGL.h>

#include <vector>

class GLObject
{
public:
    void setData(const FMatrix& q, int n, int dim, const std::vector<Vector3f>& normals, const std::vector<int>& indices)
    {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, n * dim * sizeof(float), q.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, dim, GL_FLOAT, GL_FALSE, 0, 0);
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

    GLuint vao;
    GLuint vbo;
    GLuint nbo;
    GLuint ibo;
};

class Scene
{
public:
    GLObject obj;
    GLObject floor;
};
