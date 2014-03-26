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

    corridormap::memory_malloc mem;

    float obstacle_verts_x[] = { 10.f, 90.f, 90.f, 10.f };
    float obstacle_verts_y[] = { 10.f, 10.f, 90.f, 90.f };
    int num_poly_verts = 4;

    corridormap::footprint obstacles;
    obstacles.x = obstacle_verts_x;
    obstacles.y = obstacle_verts_y;
    obstacles.num_polys = 1;
    obstacles.num_verts = 4;
    obstacles.num_poly_verts = &num_poly_verts;

    corridormap::bbox2 obstacle_bounds = corridormap::bounds(obstacles);

    const float max_dist = corridormap::max_distance(obstacle_bounds);
    const float max_error = 0.1f;

    corridormap::distance_mesh mesh = corridormap::allocate_distance_mesh(&mem, obstacles, max_dist, max_error);
    corridormap::build_distance_mesh(obstacles, mesh, max_dist, max_error);

    corridormap::renderer_gl render_iface;
    corridormap::renderer::parameters render_params;
    render_params.render_target_width = screen_width;
    render_params.render_target_height = screen_height;
    render_params.min[0] = obstacle_bounds.min[0];
    render_params.min[1] = obstacle_bounds.min[1];
    render_params.max[0] = obstacle_bounds.max[0];
    render_params.max[1] = obstacle_bounds.max[1];
    render_params.far_plane = max_dist;

    if (!render_iface.initialize(render_params))
    {
        fprintf(stderr, "failed to initialize render interface.\n");
        render_iface.finalize();
        return 1;
    }

    while (!glfwWindowShouldClose(window))
    {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        corridormap::render_distance_mesh(&render_iface, mesh);

        glViewport(0, 0, width, height);
        glClearColor(1.f, 1.f, 1.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);

        render_iface.blit_frame_buffer(width, height);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    render_iface.finalize();

    return 0;
}
