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
        {
            Mesh c1;
            loadObject("Resources/IcosphereHigh.obj", c1);
            Mesh c2;
            loadObject("Resources/IcosphereHigh.obj", c2);

            for (Vector3f& v : c1.vertices)
            {
                v.x += 1.0;
            }
            for (Vector3f& v : c2.vertices)
            {
                v.x -= 1.2f;
            }
            storeVertices(c1, vertices);
            storeVertices(c2, vertices);
            storeNormals(c1, normals);
            storeNormals(c2, normals);
            storeIndices(c1, indices, 0);
            storeIndices(c2, indices, c1.vertices.size());

            scene.cells.push_back(c1);
            scene.cells.push_back(c2);
        }
        scene.storePositions();
        solver.setPoints(scene.positions);

        makeCellConstraint(scene.cells[0], solver, 0);
        makeCellConstraint(scene.cells[1], solver, scene.cells[0].vertices.size());

        forces.resize(scene.positions.cols());

        computeNormals(scene.cells[0]);
        computeNormals(scene.cells[1]);

        // Forces
        auto gravityForce = std::make_shared<ShapeOp::GravityForce>(ShapeOp::Vector3(0, -0.05, 0));
        solver.addForces(gravityForce);

        vertexToForcesMap.resize(scene.positions.cols());

        for (int i = 0; i < scene.positions.cols(); i++)
        {
            auto vertexForce = std::make_shared<ShapeOp::VertexForce>(ShapeOp::Vector3(0, 0, 0), i);

            int id = solver.addForces(vertexForce);
            vertexToForcesMap[i] = id;
        }

        userForce = ShapeOp::Vector3(0, 0, 0);
        userForces.resize(scene.positions.cols());
        for (int i = 0; i < scene.positions.cols(); i++)
        {
            userForces[i] = std::make_shared<ShapeOp::VertexForce>(userForce, i);

            solver.addForces(userForces[i]);
        }

        // Initialize simulation
        solver.initialize(true, 0.1f, 0.96, 1.0 / 120);

        scene.velocities = solver.getVelocities();

        //simulation.init();

        scene.obj.setData(vertices, normals, indices);
        scene.floor.setFloorData();
        scene.colDebug.initializeColDebug(scene.cells[0].vertices.size());
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
            //Sleep(100);
            //simulation.update();
            
            collisionDetection();

            ///
            bool success = solver.solve(10);
            //std::cout << success << std::endl;
            scene.positions = solver.getPoints();
            scene.velocities = solver.getVelocities();

            // Store current vertices back in prev vertices
            for (int c = 0; c < scene.cells.size(); c++)
            {
                Mesh& cell = scene.cells[c];
                for (int i = 0; i < cell.vertices.size(); i++)
                {
                    cell.prevVertices[i].set(cell.vertices[i]);
                }
            }

            // Load vertex data back into mesh
            int idx = 0;
            for (Mesh& cell : scene.cells)
            {
                for (int i = 0; i < cell.vertices.size(); i++)
                {
                    cell.vertices[i].set(scene.positions(0, idx), scene.positions(1, idx), scene.positions(2, idx));
                    idx++;
                }
            }

            // Linearize data
            vertices.clear();
            for (Mesh& cell : scene.cells)
            {
                storeVertices(cell, vertices);
            }

            for (int i = 0; i < scene.cells.size(); i++)
                computeNormals(scene.cells[i]);

            for (int i = 0; i < scene.cells[0].normals.size(); i++)
                scene.cells[0].faceNormals[i] += 3;

            // Compute vertex normals
            std::vector<Vector3f> vertexNormals;
            for (int i = 0; i < scene.cells.size(); i++)
            {
                Mesh& cell = scene.cells[i];
                std::vector<Vector3f> cellVertexNormals(cell.vertices.size());

                for (int f = 0; f < cell.faces.size(); f++)
                {
                    const Face& face = cell.faces[f];
                    cellVertexNormals[face.i0] += cell.faceNormals[f];
                    cellVertexNormals[face.i1] += cell.faceNormals[f];
                    cellVertexNormals[face.i2] += cell.faceNormals[f];
                }

                vertexNormals.insert(vertexNormals.end(), cellVertexNormals.begin(), cellVertexNormals.end());
            }
            
            for (Vector3f& vertexNormal : vertexNormals)
                if (vertexNormal.x != 0 || vertexNormal.y != 0 || vertexNormal.z != 0)
                    vertexNormal.normalize();

            //CellExporter::writeMeshToFile("cell_1.csv", cell_1);
            //CellExporter::writeMeshToFile("cell_2.csv", cell_2);
            _recorder.recordFrame(scene);

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

            glLineWidth(1);
            glBindVertexArray(scene.obj.vao);
            glBindBuffer(GL_ARRAY_BUFFER, scene.obj.vbo);
            glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3f), vertices.data(), GL_STATIC_DRAW);
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

            glLineWidth(3);
            glBindVertexArray(scene.colDebug.vao);
            glBindBuffer(GL_ARRAY_BUFFER, scene.colDebug.vbo);
            glBufferData(GL_ARRAY_BUFFER, debugPositions.size() * sizeof(Vector3f), debugPositions.data(), GL_STATIC_DRAW);
            glDrawArrays(GL_POINTS, 0, debugPositions.size());

            //glBufferData(GL_ARRAY_BUFFER, debugLines.size() * sizeof(Vector3f), debugLines.data(), GL_STATIC_DRAW);
            //glDrawArrays(GL_LINES, 0, debugLines.size());

            window.update();
        }
    }

    void collisionDetection()
    {
        debugPositions.clear();

        for (Vector3f& f : forces)
            f.set(0, 0, 0);

        /// Collision detection
        // Plane
        for (int i = 0; i < scene.positions.cols(); i++)
        {
            double y = solver.getPoints()(1, i);

            if (y < -3)
            {
                float disp = -3 - y;
                forces[i] += Vector3f(0, disp * 100, 0);
            }
        }
        // Sphere
        for (int i = 0; i < scene.positions.cols(); i++)
        {
            ShapeOp::Vector3 vs = solver.getPoints().col(i);
            Vector3f vr(vs.x(), vs.y(), vs.z());

            if (vr.length() > 3)
            {
                // Test setting the position of the vertex directly
                // Project v back on sphere
                Vector3f vProj = normalize(vr) * 2.99;
                scene.positions(0, i) = vProj.x;
                scene.positions(1, i) = vProj.y;
                scene.positions(2, i) = vProj.z;
                scene.velocities(0, i) = 0;
                scene.velocities(1, i) = 0;
                scene.velocities(2, i) = 0;
                //debugPositions.push_back(v);

                // Force procedure
                //float force = v.length() - 3;
                //forces[i] += Vector3f(-v.x * 100 * force, -v.y * 100 * force, -v.z * 10 * force);
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
        //debugPositions.resize(cell_1.vertices.size());
        debugPositions.clear();
        debugLines.clear();

        cellCollision(scene.cells[0], scene.cells[1], 0);
        cellCollision(scene.cells[1], scene.cells[0], scene.cells[0].vertices.size());

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
        for (int i = 0; i < scene.positions.cols(); i++)
        {
            ShapeOp::Vector3 sforce(forces[i].x, forces[i].y, forces[i].z);
            dynamic_cast<ShapeOp::VertexForce*>(solver.getForce(vertexToForcesMap[i]).get())->setForce(sforce);
        }

        //
        //volumeMin += 0.1f;
        //volumeMax += 0.1f;
        //for (int i = 0; i < volumeConstraintIds.size(); i++)
        //{
        //    dynamic_cast<ShapeOp::VolumeConstraint*>(s.getConstraint(volumeConstraintIds[i]).get())->setRangeMin(volumeMin);
        //    dynamic_cast<ShapeOp::VolumeConstraint*>(s.getConstraint(volumeConstraintIds[i]).get())->setRangeMax(volumeMax);
        //}
        solver.setPoints(scene.positions);
        solver.setVelocities(scene.velocities);
    }

    void cellCollision(Mesh& c1, Mesh& c2, int offset)
    {
        for (int i = 0; i < c1.vertices.size(); i++)
        {
            Vector3f& v = c1.vertices[i];

            float t = 0;
            bool inCell = vertexInCell(v, c2, t);

            if (inCell)
            {
                Vector3f& pv = c1.prevVertices[i];
                Vector3f vel = v - pv;

                Vector3f closestPoint = findClosestPointOnMesh(v, c2);

                scene.positions(0, i + offset) = closestPoint.x;
                scene.positions(1, i + offset) = closestPoint.y;
                scene.positions(2, i + offset) = closestPoint.z;
                scene.velocities(0, i + offset) = 0;
                scene.velocities(1, i + offset) = 0;
                scene.velocities(2, i + offset) = 0;
            }
        }
    }

    void cellCollision2(Mesh& c1, Mesh& c2)
    {
        for (int i = 0; i < c1.vertices.size(); i++)
        {
            Vector3f& v = c1.vertices[i];

            float t = 0;
            bool inCell = vertexInCell(v, c2, t);

            if (inCell)
            {
                Vector3f& pv = c1.prevVertices[i];
                Vector3f vel = v - pv;

                Vector3f closestPoint = findClosestPointOnMesh(v, c2);

                scene.positions(0, i) = closestPoint.x;
                scene.positions(1, i) = closestPoint.y;
                scene.positions(2, i) = closestPoint.z;
                scene.velocities(0, i) = 0;
                scene.velocities(1, i) = 0;
                scene.velocities(2, i) = 0;
            }
        }

        for (int i = 0; i < c2.vertices.size(); i++)
        {
            Vector3f& v = c1.vertices[i];

            float t = 0;
            bool inCell = vertexInCell(v, c2, t);

            if (inCell)
            {
                Vector3f& pv = c1.prevVertices[i];
                Vector3f vel = v - pv;

                Vector3f closestPoint = findClosestPointOnMesh(v, c2);

                scene.positions(0, i) = closestPoint.x;
                scene.positions(1, i) = closestPoint.y;
                scene.positions(2, i) = closestPoint.z;
                scene.velocities(0, i) = 0;
                scene.velocities(1, i) = 0;
                scene.velocities(2, i) = 0;
            }
        }
    }

    void cellCollisionFull(Mesh& c1, Mesh& c2)
    {
        for (int i = 0; i < c1.vertices.size(); i++)
        {
            Vector3f& v = c1.vertices[i];
            //ShapeOp::ClosenessConstraint* c = dynamic_cast<ShapeOp::ClosenessConstraint*>(s.getConstraint(cell_1.collisionConstraintIds[i]).get());
            //c->setWeight(1.0f);

            //debugPositions[i].set(100, 100, 100);

            float t = 0;
            bool inCell = vertexInCell(v, c2, t);


            if (inCell)
            {
                //debugPositions[i].set(v + Vector3f(0, 1, 0) * t);
                Vector3f& pv = c1.prevVertices[i];
                Vector3f vel = v - pv;
                //// Determine intersection point
                //{
                //    Vector3f vel = v - pv;

                //    float minT = 100;
                //    int fHit = -1;
                //    Vector3f dd;
                //    for (int f = 0; f < cell_2.faces.size(); f++)
                //    {
                //        float t = 100;
                //        bool hit = triangleIntersect(v, -normalize(vel), cell_2, cell_2.faces[f], t);

                //        if (hit && t < minT && t > 0)
                //        {
                //            minT = t;
                //            fHit = f;
                //            dd = -normalize(vel) * t;
                //        }

                //        t = 100;
                //        hit = triangleIntersect(v, normalize(vel), cell_2, cell_2.faces[f], t);

                //        if (hit && t < minT && t > 0)
                //        {
                //            minT = t;
                //            fHit = f;
                //            dd = normalize(vel) * t;
                //        }
                //    }

                //    Vector3f x = v + dd;
                //    debugPositions.push_back(x);
                //    debugLines.push_back(v);
                //    debugLines.push_back(x);

                //    //float dist = (x - v).length();
                //    //Vector3f pushDir = (x - v) * 1000 * dist;
                //    //forces[i] += pushDir;
                //}

                Vector3f closestPoint = findClosestPointOnMesh(v, c2);
                //float dist = (closestVertex - v).length();
                //Vector3f pushDir = (closestVertex - v) * 100000 * dist;
                //forces[i] += pushDir;

                scene.positions(0, i) = closestPoint.x;
                scene.positions(1, i) = closestPoint.y;
                scene.positions(2, i) = closestPoint.z;
                scene.velocities(0, i) = 0;
                scene.velocities(1, i) = 0;
                scene.velocities(2, i) = 0;

                //std::cout << "In Cell " << dist << std::endl;
                //c->setPosition(ShapeOp::Vector3(closestVertex.x, closestVertex.y, closestVertex.z));
                //c->setWeight(5.0f);
                //std::cout << "Setting position" << std::endl;
                //debugPositions[i].set(closestVertex);
            }
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

    ShapeOp::Solver solver;
    ShapeOp::Scalar weight = 1.0;

    std::vector<int> vertexToForcesMap;
    std::vector<std::shared_ptr<ShapeOp::VertexForce>> userForces;

    std::vector<Vector3f> vertices;
    std::vector<Vector3f> normals;
    std::vector<int> indices;

    std::vector<Vector3f> debugPositions;
    std::vector<Vector3f> debugLines;

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
