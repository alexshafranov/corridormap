//
// Copyright (c) 2014 Alexander Shafranov <shafranov@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdio.h>
#include <GL/glew.h>
#include <clew.h>

#ifdef _WIN32
    #include <GL/wglew.h>
#endif

#include <GLFW/glfw3.h>
#include "corridormap/memory.h"
#include "corridormap/build.h"
#include "corridormap/render_gl.h"

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
    const int screen_width = 720;
    const int screen_height = 720;

    // colors to use for diagram instead of indices.
    unsigned colors[] =
    {
        0xffff0000, 0xff00ff00, 0xff0000ff, 0xffffff00, 0xffff00ff, 0xff00ffff, 0xff7f7f7f,
        0xff800000, 0xff008000, 0xff000080, 0xff808000, 0xff800080, 0xff008080, 0xff808080,
        0xffc00000, 0xff00c000, 0xff0000c0, 0xffc0c000, 0xffc000c0, 0xff00c0c0, 0xffc0c0c0,
        0xff400000, 0xff004000, 0xff000040, 0xff404000, 0xff400040, 0xff004040, 0xff404040,
        0xff200000, 0xff002000, 0xff000020, 0xff202000, 0xff200020, 0xff002020, 0xff202020,
        0xff600000, 0xff006000, 0xff000060, 0xff606000, 0xff600060, 0xff006060, 0xff606060,
        0xffa00000, 0xff00a000, 0xff0000a0, 0xffa0a000, 0xffa000a0, 0xff00a0a0, 0xffa0a0a0,
        0xffe00000, 0xff00e000, 0xff0000e0, 0xffe0e000, 0xffe000e0, 0xff00e0e0, 0xffe0e0e0,
    };
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

    if (clewInit("OpenCL.dll") != CLEW_SUCCESS)
    {
        return 1;
    }

    const unsigned char* vendor = glGetString(GL_VENDOR);
    const unsigned char* version = glGetString(GL_VERSION);

    printf("opengl vendor=%s version=%s\n", vendor, version);

    corridormap::memory_malloc mem;

    float obstacle_verts_x[] = { 10.f, 50.f, 30.f,  70.f, 80.f, 90.f, 90.f, 80.f, 70.f, 60.f, 60.f,  10.f, 40.f, 40.f, 10.f,  50.f, 80.f, 70.f, };
    float obstacle_verts_y[] = { 20.f, 20.f, 50.f,  20.f, 20.f, 30.f, 40.f, 50.f, 50.f, 40.f, 30.f,  70.f, 70.f, 90.f, 90.f,  70.f, 70.f, 80.f, };
    int num_poly_verts[] = { 3, 8, 4, 3 };

    corridormap::footprint obstacles;
    obstacles.x = obstacle_verts_x;
    obstacles.y = obstacle_verts_y;
    obstacles.num_polys = 4;
    obstacles.num_verts = sizeof(obstacle_verts_x)/sizeof(obstacle_verts_x[0]);
    obstacles.num_poly_verts = num_poly_verts;

    const float border = 10.f;
    corridormap::bbox2 obstacle_bounds = corridormap::bounds(obstacles, border);

    const float max_dist = corridormap::max_distance(obstacle_bounds);
    const float max_error = 0.1f;

    corridormap::distance_mesh mesh = corridormap::allocate_distance_mesh(&mem, obstacles, max_dist, max_error);
    corridormap::build_distance_mesh(obstacles, obstacle_bounds, max_dist, max_error, mesh);
    corridormap::set_segment_colors(mesh, colors, sizeof(colors)/sizeof(colors[0]));

    corridormap::renderer_gl render_iface;
    corridormap::renderer::parameters render_params;
    render_params.render_target_width = screen_width;
    render_params.render_target_height = screen_height;
    render_params.min[0] = obstacle_bounds.min[0];
    render_params.min[1] = obstacle_bounds.min[1];
    render_params.max[0] = obstacle_bounds.max[0];
    render_params.max[1] = obstacle_bounds.max[1];
    render_params.far_plane = max_dist + 0.1f;

    if (!render_iface.initialize(render_params, &mem))
    {
        fprintf(stderr, "failed to initialize render interface.\n");
        render_iface.finalize();
        return 1;
    }

    render_iface.create_opencl_shared();

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
    corridormap::deallocate_distance_mesh(&mem, mesh);

    return 0;
}
