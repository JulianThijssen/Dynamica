#include "ProjectiveDynamics.h"
#include "NewtonRaphson.h"

#include <GDT/Window.h>
#include <GDT/OpenGL.h>
#include <GDT/Shader.h>
#include <GDT/Vector3f.h>

#include <assimp/Importer.hpp>
#include "assimp/scene.h"
#include "assimp/postprocess.h"

#include <iostream>

struct Face
{
    unsigned int i0, i1, i2;
};

int main()
{
    Window window;
    window.create("Projective Dynamics", 800, 600);
    window.enableVSync(true);
    
    ShaderProgram shader;
    try
    {
        shader.create();
        shader.addShaderFromFile(ShaderType::VERTEX, "Resources/Line.vert");
        shader.addShaderFromFile(ShaderType::FRAGMENT, "Resources/Line.frag");
        shader.build();
    }
    catch (ShaderLoadingException& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    Simulation simulation;

    newtonRaphson(-20);

    ///////////////////////////////////////////////
    Assimp::Importer importer;

    unsigned int flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType;
    const aiScene* aiScene = importer.ReadFile("Resources/Icosphere.obj", flags);
    
    if (!aiScene)
    {
        std::cout << "Failed to load model: " << importer.GetErrorString() << std::endl;
        return false;
    }

    std::vector<int> indices;

    for (int i = 0; i < aiScene->mNumMeshes; i++)
    {
        const aiMesh* aiMesh = aiScene->mMeshes[i];

        std::vector<Vector3f> vertices(aiMesh->mNumVertices);
        std::vector<Vector3f> normals(aiMesh->mNumVertices);

        std::vector<Face> faces(aiMesh->mNumFaces);

        std::cout << aiMesh->mNumVertices << " " << aiMesh->mNumFaces << std::endl;

        for (int j = 0; j < aiMesh->mNumVertices; j++)
        {
            vertices[j] = Vector3f(aiMesh->mVertices[j].x, aiMesh->mVertices[j].y, aiMesh->mVertices[j].z);
            //normals[j] = Vector3f(aiMesh->mNormals[j].x, aiMesh->mNormals[j].y, aiMesh->mNormals[j].z);
        }
        for (int j = 0; j < aiMesh->mNumFaces; j++)
        {
            const aiFace& aiFace = aiMesh->mFaces[j];
            if (aiFace.mNumIndices < 3)
                continue;
            faces[j] = Face{ aiFace.mIndices[0], aiFace.mIndices[1], aiFace.mIndices[2] };
        }

        simulation.state.q.resize(vertices.size() * 3, 1);

        // Store vertices in q
        for (int j = 0; j < vertices.size(); j++)
        {
            simulation.state.q(j * 3 + 0) = vertices[j].x;
            simulation.state.q(j * 3 + 1) = vertices[j].y;
            simulation.state.q(j * 3 + 2) = vertices[j].z;
        }

        // Store edges in constraints
        for (int j = 0; j < faces.size(); j++)
        {
            Face& face = faces[j];
            simulation.constraints.push_back(new SpringConstraint((int)face.i0, (int)face.i1, 0.3f));
            simulation.constraints.push_back(new SpringConstraint((int)face.i1, (int)face.i2, 0.3f));
            simulation.constraints.push_back(new SpringConstraint((int)face.i2, (int)face.i0, 0.3f));

            indices.push_back(face.i0);
            indices.push_back(face.i1);
            indices.push_back(face.i1);
            indices.push_back(face.i2);
            indices.push_back(face.i2);
            indices.push_back(face.i0);
        }
    }

    ///////////////////////////////////////////////

    simulation.init();

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, simulation.state.n * simulation.state.dim * sizeof(float), simulation.state.q.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, simulation.state.dim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    GLuint ibo;
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(float), indices.data(), GL_STATIC_DRAW);
    //glVertexAttribPointer(0, 1, GL_INT, GL_FALSE, 0, 0);
    //glEnableVertexAttribArray(0);

    glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
    
    float dt = 0.016f;

    while (!window.shouldClose())
    {
        float fint = 0;

        simulation.update();

        // Rendering
        glClear(GL_COLOR_BUFFER_BIT);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, simulation.state.n * simulation.state.dim * sizeof(float), simulation.state.q.data(), GL_STATIC_DRAW);

        glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);

        window.update();
    }

    return 0;
}
