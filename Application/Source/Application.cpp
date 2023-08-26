#include "ProjectiveDynamics.h"
#include "NewtonRaphson.h"
#include "Scene.h"
#include "ObjLoader.h"
#include "CellExporter.h"
#include "Recorder.h"

#include <GDT/Window.h>
#include <GDT/OpenGL.h>
#include <GDT/Shader.h>
#include <GDT/Vector3f.h>
#include <GDT/Matrix4f.h>
#include <GDT/Maths.h>

#include <iostream>
#include <windows.h>

#include "ShapeOp/Solver.h"
#include "ShapeOp/Constraint.h"
#include "ShapeOp/Force.h"

class Application : public MouseMoveListener, KeyListener
{
public:
    void init()
    {
        window.create("Projective Dynamics", 1200, 1200);
        window.enableVSync(false);

        window.addMouseMoveListener(this);
        window.addKeyListener(this);

        // Shaders
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
            return;
        }

        // Solver
        newtonRaphson(-20);

        // Objects
        loadObject("Resources/IcosphereHigh.obj", cell_1);
        loadObject("Resources/IcosphereHigh.obj", cell_2);

        _recorder.addMeshObservation(&cell_1);
        _recorder.addMeshObservation(&cell_2);

        for (Vector3f& v : cell_1.vertices)
        {
            v.x += 1.0;
        }
        for (Vector3f& v : cell_2.vertices)
        {
            v.x -= 1.2f;
        }
        storeVertices(cell_1, vertices);
        storeVertices(cell_2, vertices);
        storeNormals(cell_1, normals);
        storeNormals(cell_2, normals);
        storeIndices(cell_1, indices, 0);
        storeIndices(cell_2, indices, cell_1.vertices.size());

        verticesToSimulation(vertices, p, s);

        s.setPoints(p);
        makeCellConstraint(cell_1, s, 0);
        makeCellConstraint(cell_2, s, cell_1.vertices.size());

        forces.resize(p.cols());

        computeNormals(cell_1);
        computeNormals(cell_2);

        // Forces
        auto gravityForce = std::make_shared<ShapeOp::GravityForce>(ShapeOp::Vector3(0, -0.05, 0));
        s.addForces(gravityForce);

        vertexToForcesMap.resize(p.cols());

        for (int i = 0; i < p.cols(); i++)
        {
            auto vertexForce = std::make_shared<ShapeOp::VertexForce>(ShapeOp::Vector3(0, 0, 0), i);

            int id = s.addForces(vertexForce);
            vertexToForcesMap[i] = id;
        }

        userForce = ShapeOp::Vector3(0, 0, 0);
        userForces.resize(p.cols());
        for (int i = 0; i < p.cols(); i++)
        {
            userForces[i] = std::make_shared<ShapeOp::VertexForce>(userForce, i);

            s.addForces(userForces[i]);
        }

        // Initialize simulation
        s.initialize(true, 0.1f, 0.96, 1.0 / 120);

        //simulation.init();

        scene.obj.setData(vertices, normals, indices);
        scene.floor.setFloorData();
        scene.colDebug.initializeColDebug(cell_1.vertices.size());
    }
    
    void update()
    {
        glEnable(GL_DEPTH_TEST);
        glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
        glPointSize(10.0f);

        float dt = 0.0016f;
        float t = 0;

        Sleep(500);
        
        float volumeMin = 0.95;
        float volumeMax = 1.05;

        while (!window.shouldClose())
        {
            float fint = 0;
            //Sleep(100);
            //simulation.update();
            
            for (Vector3f& f : forces)
                f.set(0, 0, 0);

            /// Collision detection
            // Plane
            for (int i = 0; i < p.cols(); i++)
            {
                double y = s.getPoints()(1, i);

                if (y < -3)
                {
                    float disp = -3 - y;
                    forces[i] += Vector3f(0, disp * 100, 0);
                }
            }
            // Sphere
            for (int i = 0; i < p.cols(); i++)
            {
                ShapeOp::Vector3 v = s.getPoints().col(i);

                if (v.norm() > 3)
                {
                    float force = v.norm() - 3;
                    forces[i] += Vector3f(-v(0) * 100 * force, -v(1) * 100 * force, -v(2) * 10 * force);
                }
            }
            // Internal collision
            //for (int i = 0; i < p.cols()-1; i++)
            //{
            //    for (int j = i+1; j < p.cols(); j++)
            //    {
            //        Vector3f v0(s.getPoints()(0, i), s.getPoints()(1, i), s.getPoints()(2, i));
            //        Vector3f v1(s.getPoints()(0, j), s.getPoints()(1, j), s.getPoints()(2, j));

            //        float dist = (v1 - v0).length();
            //        if (dist == 0) continue;
            //        Vector3f dir = (v1 - v0).normalize();

            //        if (dist < 0.1)
            //        {
            //            forces[i] += -dir * 1000;
            //            forces[j] += dir * 1000;
            //        }
            //    }
            //}
            // 
            
            // Test if vertices are in other cells
            // Cell 1
            std::vector<Vector3f> debugPositions(cell_1.vertices.size());
            for (int i = 0; i < cell_1.vertices.size(); i++)
            {
                Vector3f& v = cell_1.vertices[i];
                //ShapeOp::ClosenessConstraint* c = dynamic_cast<ShapeOp::ClosenessConstraint*>(s.getConstraint(cell_1.collisionConstraintIds[i]).get());
                //c->setWeight(1.0f);

                debugPositions[i].set(100, 100, 100);

                float t = 0;
                bool inCell = vertexInCell(v, cell_2, t);


                if (inCell)
                {
                    //debugPositions[i].set(v + Vector3f(0, 1, 0) * t);
                    Vector3f& pv = cell_1.prevVertices[i];
                    Vector3f vel = v - pv;
                    // Determine intersection point
                    {
                        Vector3f vel = v - pv;

                        float minT = 100;
                        int fHit = -1;
                        for (int f = 0; f < cell_2.faces.size(); f++)
                        {
                            float t = 100;
                            bool hit = triangleIntersect(v, -normalize(vel), cell_2, cell_2.faces[f], t);
                            
                            if (hit && t < minT && t > 0)
                            {
                                minT = t;
                                fHit = f;
                            }
                        }

                        Vector3f x = v + -normalize(vel) * t;
                        debugPositions[i].set(x);
                    }
                    
                    Vector3f closestVertex = findClosestPointOnMesh(v, cell_2);
                    float dist = (closestVertex - v).length();
                    Vector3f pushDir = (closestVertex - v) * 100000 * dist;
                    forces[i] += pushDir;
                    
                    //std::cout << "In Cell " << dist << std::endl;
                    //c->setPosition(ShapeOp::Vector3(closestVertex.x, closestVertex.y, closestVertex.z));
                    //c->setWeight(5.0f);
                    //std::cout << "Setting position" << std::endl;
                    //debugPositions[i].set(closestVertex);
                }
            }

            // Cell 2
            //for (int i = 0; i < cell_2.vertices.size(); i++)
            //{
            //    Vector3f& v = cell_2.vertices[i];
            //    bool inCell = vertexInCell(v, cell_1);
            //    if (inCell)
            //    {
            //        Vector3f closestVertex = findClosestVertex(v, cell_1);

            //        Vector3f pushDir = (closestVertex - v) * 1000;
            //        forces[i + cell_1.vertices.size()] += pushDir;
            //    }
            //}

            // Set forces in simulation
            for (int i = 0; i < p.cols(); i++)
            {
                ShapeOp::Vector3 sforce(forces[i].x, forces[i].y, forces[i].z);
                dynamic_cast<ShapeOp::VertexForce*>(s.getForce(vertexToForcesMap[i]).get())->setForce(sforce);
            }

            //
            //volumeMin += 0.1f;
            //volumeMax += 0.1f;
            //for (int i = 0; i < volumeConstraintIds.size(); i++)
            //{
            //    dynamic_cast<ShapeOp::VolumeConstraint*>(s.getConstraint(volumeConstraintIds[i]).get())->setRangeMin(volumeMin);
            //    dynamic_cast<ShapeOp::VolumeConstraint*>(s.getConstraint(volumeConstraintIds[i]).get())->setRangeMax(volumeMax);
            //}

            ///
            bool success = s.solve(10);
            //std::cout << success << std::endl;
            p = s.getPoints();

            // Linearize data
            std::vector<float> vertData(p.rows() * p.cols());
            for (int i = 0; i < p.cols(); i++)
            {
                for (int d = 0; d < p.rows(); d++)
                {
                    vertData[i * 3 + d] = p(d, i);
                }
            }
            ///
            // Store current vertices back in prev vertices
            for (int i = 0; i < cell_1.vertices.size(); i++)
            {
                cell_1.prevVertices[i].set(cell_1.vertices[i]);
            }
            for (int i = 0; i < cell_2.vertices.size(); i++)
            {
                cell_2.prevVertices[i].set(cell_2.vertices[i]);
            }
            // Load vertex data back into mesh
            for (int i = 0; i < cell_1.vertices.size(); i++)
            {
                cell_1.vertices[i].set(p(0, i), p(1, i), p(2, i));
            }
            int offset = cell_1.vertices.size();
            for (int i = 0; i < cell_2.vertices.size(); i++)
            {
                cell_2.vertices[i].set(p(0, offset + i), p(1, offset + i), p(2, offset + i));
            }

            computeNormals(cell_1);
            computeNormals(cell_2);

            // Compute vertex normals
            std::vector<Vector3f> vertexNormals(cell_1.vertices.size() + cell_2.vertices.size());
            for (int i = 0; i < cell_1.faces.size(); i++)
            {
                const Face& face = cell_1.faces[i];
                vertexNormals[face.i0] += cell_1.faceNormals[i];
                vertexNormals[face.i1] += cell_1.faceNormals[i];
                vertexNormals[face.i2] += cell_1.faceNormals[i];
            }
            for (int i = 0; i < cell_2.faces.size(); i++)
            {
                const Face& face = cell_2.faces[i];
                vertexNormals[face.i0 + cell_1.vertices.size()] += cell_2.faceNormals[i];
                vertexNormals[face.i1 + cell_1.vertices.size()] += cell_2.faceNormals[i];
                vertexNormals[face.i2 + cell_1.vertices.size()] += cell_2.faceNormals[i];
            }
            for (Vector3f& vertexNormal : vertexNormals)
                if (vertexNormal.x != 0 || vertexNormal.y != 0 || vertexNormal.z != 0)
                    vertexNormal.normalize();

            //CellExporter::writeMeshToFile("cell_1.csv", cell_1);
            //CellExporter::writeMeshToFile("cell_2.csv", cell_2);
            _recorder.recordFrame();

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
            viewMatrix.translate(-Vector3f(0, 0, 10));
            viewMatrix.rotate(t*0, 0, 1, 0);

            Matrix4f modelMatrix;
            modelMatrix.setIdentity();
            //modelMatrix.rotate(t, 0, 1, 0);
            //modelMatrix.scale(0.5f);

            shader.uniformMatrix4f("projMatrix", projMatrix);
            shader.uniformMatrix4f("viewMatrix", viewMatrix);
            shader.uniformMatrix4f("modelMatrix", modelMatrix);

            // Rendering
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glBindVertexArray(scene.obj.vao);
            glBindBuffer(GL_ARRAY_BUFFER, scene.obj.vbo);
            glBufferData(GL_ARRAY_BUFFER, vertData.size() * sizeof(float), vertData.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, scene.obj.nbo);
            glBufferData(GL_ARRAY_BUFFER, vertexNormals.size() * sizeof(Vector3f), vertexNormals.data(), GL_STATIC_DRAW);
            //glBufferData(GL_ARRAY_BUFFER, simulation.state.n * simulation.state.dim * sizeof(float), simulation.state.q.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, scene.obj.ibo);
            glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);

            // Draw floor
            modelMatrix.setIdentity();
            modelMatrix.translate(Vector3f(0, -4.5, 0));
            modelMatrix.scale(Vector3f(2, 1, 2));

            shader.uniformMatrix4f("modelMatrix", modelMatrix);

            glBindVertexArray(scene.floor.vao);
            glBindBuffer(GL_ARRAY_BUFFER, scene.floor.vbo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, scene.floor.ibo);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

            // Draw col debug
            modelMatrix.setIdentity();
            shader.uniformMatrix4f("modelMatrix", modelMatrix);

            glBindVertexArray(scene.colDebug.vao);
            glBindBuffer(GL_ARRAY_BUFFER, scene.colDebug.vbo);
            glBufferData(GL_ARRAY_BUFFER, debugPositions.size() * sizeof(Vector3f), debugPositions.data(), GL_STATIC_DRAW);
            glDrawArrays(GL_POINTS, 0, cell_1.vertices.size());

            window.update();
        }
    }

    void onMouseMove(float x, float y) override
    {
        std::cout << x << " " << y << std::endl;
        float cx = window.getWidth() / 2;
        float cy = window.getHeight() / 2;

        userForce(0) = (x - cx) / 25;
        userForce(1) = -(y - cy) / 25;
        for (int i = 0; i < userForces.size(); i++)
        {
            userForces[i]->setForce(userForce);
        }
    }

    virtual void onKeyPressed(int key, int mods) override
    {

    }

    virtual void onKeyReleased(int key, int mods) override
    {
        if (key == 82)
        {
            std::cout << "Start recording" << std::endl;
            _recorder.start();
        }
        if (key == 83)
        {
            std::cout << "Stopping recording" << std::endl;
            _recorder.stop();
        }
    }

private:
    Window window;

    ShaderProgram shader;

    Scene scene;

    //Simulation simulation;

    ShapeOp::Solver s;
    ShapeOp::Matrix3X p;
    ShapeOp::Scalar weight = 1.0;

    std::vector<int> vertexToForcesMap;
    std::vector<std::shared_ptr<ShapeOp::VertexForce>> userForces;

    Mesh cell_1;
    Mesh cell_2;

    std::vector<Vector3f> vertices;
    std::vector<Vector3f> normals;
    std::vector<int> indices;

    std::vector<Vector3f> forces;

    ShapeOp::Vector3 userForce;

    Recorder _recorder;
};

int main()
{
    Application application;
    application.init();
    application.update();

    return 0;
}
