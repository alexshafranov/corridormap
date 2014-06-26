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
#include <clew_gl.h>

#ifdef _WIN32
    #include <GL/wglew.h>
#endif

#include <GLFW/glfw3.h>
#include "corridormap/memory.h"
#include "corridormap/render_gl.h"
#include "corridormap/build.h"
#include "corridormap/build_alloc.h"
#include "corridormap/build_ocl.h"

// force NVIDIA GPU when using Optimus.
#ifdef _WIN32

extern "C"
{
    __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
}

#endif

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
    const int screen_width = 1024;
    const int screen_height = 1024;

    const int render_target_width = 1024;
    const int render_target_height = 1024;

    // colors to use for diagram instead of indices.
    unsigned colors[] =
    {
        0x00000000, 0xff0000ff, 0x00ff00ff, 0x0000ffff, 0xffff00ff, 0xff00ffff, 0x00ffffff,
        0x800000ff, 0x008000ff, 0x000080ff, 0x808000ff, 0x800080ff, 0x008080ff, 0x808080ff,
        0xc00000ff, 0x00c000ff, 0x0000c0ff, 0xc0c000ff, 0xc000c0ff, 0x00c0c0ff, 0xc0c0c0ff,
        0x400000ff, 0x004000ff, 0x000040ff, 0x404000ff, 0x400040ff, 0x004040ff, 0x404040ff,
        0x200000ff, 0x002000ff, 0x000020ff, 0x202000ff, 0x200020ff, 0x002020ff, 0x202020ff,
        0x600000ff, 0x006000ff, 0x000060ff, 0x606000ff, 0x600060ff, 0x006060ff, 0x606060ff,
        0xa00000ff, 0x00a000ff, 0x0000a0ff, 0xa0a000ff, 0xa000a0ff, 0x00a0a0ff, 0xa0a0a0ff,
        0xe00000ff, 0x00e000ff, 0x0000e0ff, 0xe0e000ff, 0xe000e0ff, 0x00e0e0ff, 0xe0e0e0ff,
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

    glfw_window_context glfw_window_ctx(screen_width, screen_height, "voronoi");
    GLFWwindow* window = glfw_window_ctx.window;

    if (!window)
    {
        fprintf(stderr, "failed to create GLFW window.\n");
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glewInit();

    if (clewInit("OpenCL.dll") != CLEW_SUCCESS)
    {
        return 1;
    }

    const unsigned char* vendor = glGetString(GL_VENDOR);
    const unsigned char* version = glGetString(GL_VERSION);

    printf("opengl vendor=%s version=%s\n", vendor, version);

    corridormap::Memory_Malloc mem;

    float obstacle_verts_x[] = { 10.f, 50.f, 30.f,  70.f, 80.f, 90.f, 90.f, 80.f, 70.f, 60.f, 60.f,  10.f, 40.f, 40.f, 10.f,  50.f, 80.f, 70.f, };
    float obstacle_verts_y[] = { 20.f, 20.f, 50.f,  20.f, 20.f, 30.f, 40.f, 50.f, 50.f, 40.f, 30.f,  70.f, 70.f, 90.f, 90.f,  70.f, 70.f, 80.f, };
    int num_poly_verts[] = { 3, 8, 4, 3 };

    corridormap::Footprint obstacles;
    obstacles.x = obstacle_verts_x;
    obstacles.y = obstacle_verts_y;
    obstacles.num_polys = sizeof(num_poly_verts)/sizeof(num_poly_verts[0]);
    obstacles.num_verts = sizeof(obstacle_verts_x)/sizeof(obstacle_verts_x[0]);
    obstacles.num_poly_verts = num_poly_verts;

    const float border = 10.f;
    corridormap::Bbox2 obstacle_bounds = corridormap::bounds(obstacles, border);

    const float max_dist = corridormap::max_distance(obstacle_bounds);
    const float max_error = 0.1f;

    corridormap::Distance_Mesh mesh = corridormap::allocate_distance_mesh(&mem, obstacles.num_polys, max_distance_mesh_verts(obstacles, max_dist, max_error));
    corridormap::build_distance_mesh(obstacles, obstacle_bounds, max_dist, max_error, mesh);
    corridormap::set_segment_colors(mesh, colors, sizeof(colors)/sizeof(colors[0]));

    corridormap::Renderer_GL render_iface;
    corridormap::Renderer::Parameters render_params;
    render_params.render_target_width = render_target_width;
    render_params.render_target_height = render_target_height;
    render_params.min[0] = obstacle_bounds.min[0];
    render_params.min[1] = obstacle_bounds.min[1];
    render_params.max[0] = obstacle_bounds.max[0];
    render_params.max[1] = obstacle_bounds.max[1];
    render_params.far_plane = max_dist + 0.1f;

    if (!render_iface.initialize(render_params, &mem))
    {
        fprintf(stderr, "failed to initialize render interface.\n");
        return 1;
    }

    corridormap::render_distance_mesh(&render_iface, mesh);

    corridormap::Renderer::Opencl_Shared cl_shared = render_iface.create_opencl_shared();
    corridormap::Opencl_Runtime cl_runtime = corridormap::init_opencl_runtime(cl_shared);

    // build kernels.
    {
        corridormap::Compilation_Status status = corridormap::build_kernels(cl_runtime);

        if (status.kernel != corridormap::kernel_id_count)
        {
            fprintf(stderr, "failed to build kernels: code=%d, kernel=%d\n", status.code, status.kernel);
            size_t build_log_size;
            clGetProgramBuildInfo(cl_runtime.programs[status.kernel], cl_runtime.device, CL_PROGRAM_BUILD_LOG, 0, 0, &build_log_size);
            char* build_log = corridormap::allocate<char>(&mem, build_log_size);
            clGetProgramBuildInfo(cl_runtime.programs[status.kernel], cl_runtime.device, CL_PROGRAM_BUILD_LOG, build_log_size, build_log, 0);
            fprintf(stderr, "build log: %s\n", build_log);
            return 1;
        }
    }

    {
        cl_int error_code;
        cl_mem voronoi_image = render_iface.share_pixels(cl_runtime.context, CL_MEM_READ_WRITE, &error_code);

        if (error_code != CL_SUCCESS)
        {
            fprintf(stderr, "failed to create opencl voronoi image.\n");
            return 1;
        }

        render_iface.acquire_shared(cl_runtime.queue, voronoi_image);
        error_code = corridormap::mark_voronoi_features(cl_runtime, voronoi_image);
        error_code = corridormap::debug_voronoi_features(cl_runtime, voronoi_image, cl_runtime.voronoi_edges_img, 0xff000000, 0);
        error_code = corridormap::debug_voronoi_features(cl_runtime, voronoi_image, cl_runtime.voronoi_vertices_img, 0xffffffff, 8);
        error_code = corridormap::compact_voronoi_features(cl_runtime);
        error_code = corridormap::store_obstacle_ids(cl_runtime, voronoi_image);
        render_iface.release_shared(cl_runtime.queue, voronoi_image);

        clFinish(cl_runtime.queue);

        corridormap::Voronoi_Features features = corridormap::allocate_voronoi_features(&mem, render_target_width, render_target_height, cl_runtime.voronoi_vertex_mark_count, cl_runtime.voronoi_edge_mark_count);
        error_code = corridormap::transfer_voronoi_features(cl_runtime, features);

        clFinish(cl_runtime.queue);

        if (error_code != CL_SUCCESS)
        {
            fprintf(stderr, "failed to run opencl kernels.\n");
            return 1;
        }

        printf("voronoi vertices: %d\n", cl_runtime.voronoi_vertex_mark_count);
        printf("voronoi edge marks: %d\n", cl_runtime.voronoi_edge_mark_count);
    }

    while (!glfwWindowShouldClose(window))
    {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        render_iface.blit_frame_buffer(width, height);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    corridormap::deallocate(&mem, mesh);

    return 0;
}
