#include "ProjectiveDynamics.h"
#include "NewtonRaphson.h"
#include "Scene.h"

#include <GDT/Window.h>
#include <GDT/OpenGL.h>
#include <GDT/Shader.h>
#include <GDT/Vector3f.h>
#include <GDT/Matrix4f.h>
#include <GDT/Maths.h>

#include <assimp/Importer.hpp>
#include "assimp/scene.h"
#include "assimp/postprocess.h"

#include <iostream>
#include <windows.h>

struct Face
{
    unsigned int i0, i1, i2;
};

int main()
{
    Window window;
    window.create("Projective Dynamics", 800, 600);
    window.enableVSync(false);
    
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

    Scene scene{};

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

    std::vector<Vector3f> normals;
    std::vector<int> indices;

    for (int i = 0; i < aiScene->mNumMeshes; i++)
    {
        const aiMesh* aiMesh = aiScene->mMeshes[i];
        std::cout << "Num vertices: " << aiMesh->mNumVertices << std::endl;
        std::vector<Vector3f> vertices(aiMesh->mNumVertices+1);
        normals.resize(aiMesh->mNumVertices+1);

        std::vector<Face> faces(aiMesh->mNumFaces);

        std::cout << aiMesh->mNumVertices << " " << aiMesh->mNumFaces << std::endl;
        
        for (int j = 0; j < aiMesh->mNumVertices; j++)
        {
            vertices[j] = Vector3f(aiMesh->mVertices[j].x, aiMesh->mVertices[j].y, aiMesh->mVertices[j].z);
            normals[j] = vertices[j]; // Vector3f(aiMesh->mNormals[j].x, aiMesh->mNormals[j].y, aiMesh->mNormals[j].z);
        }
        vertices[aiMesh->mNumVertices] = Vector3f(0, 0, 0);
        normals[aiMesh->mNumVertices] = Vector3f(0, 0, 0);

        for (int j = 0; j < aiMesh->mNumFaces; j++)
        {
            const aiFace& aiFace = aiMesh->mFaces[j];
            if (aiFace.mNumIndices < 3)
                continue;
            faces[j] = Face{ aiFace.mIndices[0], aiFace.mIndices[1], aiFace.mIndices[2] };
        }

        simulation.state.q.resize(vertices.size() * 3, 1);
        std::cout << simulation.state.q << std::endl;
        // Store vertices in q
        for (int j = 0; j < vertices.size(); j++)
        {
            simulation.state.q(j * 3 + 0) = vertices[j].x;
            simulation.state.q(j * 3 + 1) = vertices[j].y;
            simulation.state.q(j * 3 + 2) = vertices[j].z;
        }

        std::cout << "AFTER" << std::endl;
        std::cout << simulation.state.q << std::endl;
        std::cout << "AFTER2" << std::endl;
        // Store edges in constraints
        for (int j = 0; j < faces.size(); j++)
        {
            Face& face = faces[j];
            //simulation.constraints.push_back(new SpringConstraint((int)face.i0, (int)face.i1, 0.3f));
            //simulation.constraints.push_back(new SpringConstraint((int)face.i1, (int)face.i2, 0.3f));
            //simulation.constraints.push_back(new SpringConstraint((int)face.i2, (int)face.i0, 0.3f));
            std::cout << "Face: " << face.i0 << " " << face.i1 << " " << face.i2 << std::endl;
            simulation.constraints.push_back(new TetConstraint(face.i0, face.i1, face.i2, aiMesh->mNumVertices, simulation.state.q));

            indices.push_back(face.i0);
            indices.push_back(face.i1);
            indices.push_back(face.i1);
            indices.push_back(face.i2);
            indices.push_back(face.i2);
            indices.push_back(face.i0);

            indices.push_back(face.i0);
            indices.push_back(aiMesh->mNumVertices);
            indices.push_back(face.i1);
            indices.push_back(aiMesh->mNumVertices);
            indices.push_back(face.i2);
            indices.push_back(aiMesh->mNumVertices);
        }
    }

    ///////////////////////////////////////////////

    simulation.init();

    scene.obj.setData(simulation.state.q, simulation.state.n, simulation.state.dim, normals, indices);
    scene.floor.setFloorData();

    glEnable(GL_DEPTH_TEST);
    glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
    
    float dt = 0.0016f;
    float t = 0;

    Sleep(500);

    while (!window.shouldClose())
    {
        float fint = 0;

        simulation.update();

        shader.bind();

        t += dt * 10;

        Matrix4f projMatrix;
        projMatrix.setIdentity();
        float fovyr = Math::toRadians(60);
        float aspect = 1.0f;
        float zNear = 0.1f;
        float zFar = 100;
        projMatrix[0] = (float)(1 / tan(fovyr / 2)) / aspect;
        projMatrix[5] = (float)(1 / tan(fovyr / 2));
        projMatrix[10] = (zNear + zFar) / (zNear - zFar);
        projMatrix[11] = -1;
        projMatrix[14] = (2 * zNear * zFar) / (zNear - zFar);
        projMatrix[15] = -0;

        Matrix4f viewMatrix;
        viewMatrix.setIdentity();
        viewMatrix.rotate(20, 1, 0, 0);
        viewMatrix.translate(-Vector3f(0, 1, 3));

        Matrix4f modelMatrix;
        modelMatrix.setIdentity();
        //modelMatrix.rotate(t, 0, 1, 0);
        modelMatrix.scale(0.5f);

        shader.uniformMatrix4f("projMatrix", projMatrix);
        shader.uniformMatrix4f("viewMatrix", viewMatrix);
        shader.uniformMatrix4f("modelMatrix", modelMatrix);

        // Rendering
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glBindVertexArray(scene.obj.vao);
        glBindBuffer(GL_ARRAY_BUFFER, scene.obj.vbo);
        glBufferData(GL_ARRAY_BUFFER, simulation.state.n * simulation.state.dim * sizeof(float), simulation.state.q.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, scene.obj.ibo);
        glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);

        modelMatrix.setIdentity();
        modelMatrix.translate(Vector3f(0, -1.5f, 0));
        modelMatrix.scale(Vector3f(2, 1, 2));

        shader.uniformMatrix4f("projMatrix", projMatrix);
        shader.uniformMatrix4f("viewMatrix", viewMatrix);
        shader.uniformMatrix4f("modelMatrix", modelMatrix);

        glBindVertexArray(scene.floor.vao);
        glBindBuffer(GL_ARRAY_BUFFER, scene.floor.vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, scene.floor.ibo);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        window.update();
    }

    return 0;
}
