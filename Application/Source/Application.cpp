#include "ProjectiveDynamics.h"
#include "NewtonRaphson.h"

#include <GDT/Window.h>
#include <GDT/OpenGL.h>
#include <GDT/Shader.h>

#include <iostream>

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
    simulation.init();

    newtonRaphson(-20);

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, simulation.state.n * simulation.state.dim * sizeof(float), simulation.state.q.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, simulation.state.dim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

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

        glDrawArrays(GL_LINES, 0, 2);

        window.update();
    }

    return 0;
}
