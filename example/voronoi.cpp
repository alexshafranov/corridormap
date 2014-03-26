#include <stdio.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "render_gl.h"
#include "corridormap/memory.h"
#include "corridormap/build.h"

struct glfw_context
{
    bool ok;

    glfw_context()
    {
        ok = (glfwInit() == GL_TRUE);
    }

    ~glfw_context()
    {
        glfwTerminate();
    }
};

struct glfw_window_context
{
    GLFWwindow* window;

    glfw_window_context(int width, int height, const char* title)
    {
        window = glfwCreateWindow(width, height, title, 0, 0);
    }

    ~glfw_window_context()
    {
        glfwDestroyWindow(window);
    }
};

namespace
{
    const int screen_width = 1280;
    const int screen_height = 720;
}

int main()
{
    glfw_context glfw_ctx;

    if (!glfw_ctx.ok)
    {
        fprintf(stderr, "failed to initialize GLFW context.\n");
        return 1;
    }

    glfw_window_context glfw_window_ctx(screen_width, screen_height, "Voronoi");
    GLFWwindow* window = glfw_window_ctx.window;

    if (!window)
    {
        fprintf(stderr, "failed to create GLFW window.\n");
        return 1;
    }

    glfwMakeContextCurrent(window);

    glewInit();

    printf("%s\n", glGetString(GL_VERSION));

    corridormap::renderer_gl render_iface;
    corridormap::renderer::parameters render_params;
    render_params.render_target_width = screen_width;
    render_params.render_target_height = screen_height;
    render_params.min[0] = 0.f;
    render_params.min[1] = 0.f;
    render_params.max[0] = 100.f;
    render_params.max[1] = 100.f;
    // render_params.far_plane = corridormap::max_distance(render_params.min, render_params.max);
    render_params.far_plane = 100.f;

    // float obstacle_verts[8] = { 10.f, 10.f,  90.f, 10.f,  90.f, 90.f,  10.f, 90.f };
    // corridormap::polygon obstacle;
    // obstacle.vertices = obstacle_verts;
    // obstacle.nverts = 4;

    // corridormap::memory_malloc mem;
    // corridormap::triangle_list distance_mesh = corridormap::build_distance_mesh(obstacle, render_params.far_plane, 0.1f, &mem);

    float obstacle_verts[] =
    {
        0.f,    0.f,    0.f,
        100.f,  0.f,    0.f,
        0.f,    100.f,  0.f,
    };

    if (!render_iface.initialize(render_params))
    {
        fprintf(stderr, "failed to initialize render interface.\n");
        render_iface.finalize();
        return 1;
    }

    while (!glfwWindowShouldClose(window))
    {
        float ratio;
        int width, height;

        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (float) height;

        glViewport(0, 0, width, height);
        glClearColor(1.f, 1.f, 1.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        render_iface.begin();
        // render_iface.draw(distance_mesh.vertices, distance_mesh.ntris, 0xffff0000, corridormap::renderer::primitive_type_list);
        render_iface.draw(obstacle_verts, 1, 0xffff0000, corridormap::renderer::primitive_type_list);
        render_iface.end();
        render_iface.blit_frame_buffer(width, height);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    render_iface.finalize();

    return 0;
}
