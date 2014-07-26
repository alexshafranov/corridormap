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
#define CORRIDORMAP_ENABLE_OPENCL_SHARING
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

    float obstacle_verts_x[] = {
        546.04233f, 586.87983f, 586.87983f, 546.04233f, 484.7861f, 443.9486f, 443.9486f, 484.7861f, 219.27517f, 299.49779f, 367.03349f, 286.81087f, 461.04229f, 567.87837f, 549.26637f, 442.43029f, 655.31886f,
        757.95437f, 790.9898f, 688.35429f, 103.76307f, 307.49818f, 332.23693f, 128.50182f, 482.48372f, 665.65059f, 633.51629f, 450.34941f, 862.14993f, 778.93614f, 719.02164f, 802.23542f, 87.27517f, 167.49779f,
        235.03349f, 154.81087f, 359.17335f, 439.50653f, 507.13531f, 426.80213f, 886.58172f, 791.09332f, 761.96796f, 792.52284f, 828.33099f, 845.80621f, 952.58172f, 857.09332f, 827.96796f, 858.52284f, 894.33099f,
        911.80621f, 83.84732f, 221.04656f, 272.15267f, 134.95344f, 159.50583f, 175.66538f, 165.88157f, 130.15439f, 91.23934f, 75.07979f, 84.8636f, 120.59078f,
    };

    float obstacle_verts_y[] = {
        366.49453f, 413.80088f, 461.10723f, 508.41357f, 508.41357f, 461.10723f, 413.80088f, 366.49453f, 276.47356f, 203.07432f, 276.8883f, 350.28754f, 135.41626f, 155.64419f, 253.9456f, 233.71767f, 259.41453f,
        223.51093f, 317.94733f, 353.85093f, 639.66596f, 563.66847f, 622.46545f, 698.46295f, 602.91856f, 720.10861f, 775.21285f, 658.02281f, 470.77478f, 673.8586f, 651.35665f, 448.27281f, 432.47356f, 359.07432f,
        432.8883f, 506.28754f, 868.83723f, 823.54104f, 869.09317f, 914.38936f, 901.90592f, 949.29104f, 890.59889f, 802.29103f, 784.52161f, 819.73691f, 219.90592f, 267.29104f, 208.59889f, 120.29103f, 102.52161f,
        137.73691f, 914.32913f, 745.62826f, 783.80228f, 952.5032f, 122.51486f, 161.9206f, 193.21563f, 216.39993f, 204.23386f, 164.82812f, 133.53309f, 110.34879f,
    };

    int num_poly_verts[] = { 8, 4, 4, 4, 4, 4, 4, 4, 4, 6, 6, 4, 8, };

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
        error_code = corridormap::debug_voronoi_features(cl_runtime, voronoi_image, cl_runtime.voronoi_vertices_img, 0xffffffff, 4);
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
