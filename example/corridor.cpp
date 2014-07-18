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

#include "nanovg.h"
#define NANOVG_GL3_IMPLEMENTATION
#include "nanovg_gl.h"

#include "corridormap/memory.h"
#include "corridormap/render_gl.h"
#include "corridormap/build.h"
#include "corridormap/runtime.h"
#include "corridormap/vec2.h"
#include "corridormap/build_alloc.h"
#include "corridormap/build_ocl.h"
#include "draw.h"

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
}

int main()
{
    glfw_context glfw_ctx;

    if (!glfw_ctx.ok)
    {
        fprintf(stderr, "failed to initialize GLFW context.\n");
        return 1;
    }

    glfw_window_context glfw_window_ctx(screen_width, screen_height, "corridormap");
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

    NVGcontext* vg = nvgCreateGL3(512, 512, NVG_ANTIALIAS);

    if (!vg)
    {
        printf("failed to init nanovg.\n");
        return 1;
    }

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

    const float border = 60.f;
    corridormap::Bbox2 obstacle_bounds = corridormap::fit(corridormap::bounds(obstacles, border), float(render_target_width)/float(render_target_height));

    const float max_dist = corridormap::max_distance(obstacle_bounds);
    const float max_error = 0.1f;

    corridormap::Footprint_Normals normals = corridormap::allocate_foorprint_normals(&mem, obstacles.num_polys, obstacles.num_verts);
    corridormap::build_footprint_normals(obstacles, obstacle_bounds, normals);

    corridormap::Distance_Mesh mesh = corridormap::allocate_distance_mesh(&mem, obstacles.num_polys, max_distance_mesh_verts(obstacles, max_dist, max_error));
    corridormap::build_distance_mesh(obstacles, obstacle_bounds, max_dist, max_error, mesh);

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

    corridormap::Voronoi_Features features;
    corridormap::Voronoi_Traced_Edges traced_edges;
    corridormap::Walkable_Space space;
    corridormap::Corridor corridor;

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
        error_code = corridormap::compact_voronoi_features(cl_runtime);
        error_code = corridormap::store_obstacle_ids(cl_runtime, voronoi_image);
        render_iface.release_shared(cl_runtime.queue, voronoi_image);

        clFinish(cl_runtime.queue);

        features = corridormap::allocate_voronoi_features(&mem, render_target_width, render_target_height, cl_runtime.voronoi_vertex_mark_count, cl_runtime.voronoi_edge_mark_count);
        error_code = corridormap::transfer_voronoi_features(cl_runtime, features);

        clFinish(cl_runtime.queue);

        if (error_code != CL_SUCCESS)
        {
            fprintf(stderr, "failed to run opencl kernels.\n");
            return 1;
        }

        printf("voronoi vertices: %d\n", cl_runtime.voronoi_vertex_mark_count);
        printf("voronoi edge marks: %d\n", cl_runtime.voronoi_edge_mark_count);

        corridormap::Voronoi_Edge_Spans edge_spans = corridormap::allocate_voronoi_edge_spans(&mem, features.num_edge_points);
        corridormap::build_edge_spans(features, obstacles, normals, obstacle_bounds, edge_spans);

        corridormap::CSR_Grid vert_csr = corridormap::allocate_csr_grid(&mem, render_target_height, render_target_width, features.num_vert_points);
        corridormap::build_csr(features.verts, vert_csr);

        corridormap::CSR_Grid edge_csr = corridormap::allocate_csr_grid(&mem, render_target_height, render_target_width, features.num_edge_points);
        corridormap::build_csr(features.edges, edge_csr);

        traced_edges = corridormap::allocate_voronoi_traced_edges(&mem, features.num_vert_points, obstacles.num_verts);

        corridormap::trace_edges(&mem, vert_csr, edge_csr, edge_spans, features, traced_edges);
        printf("edge_count=%d\n", traced_edges.num_edges);
        printf("event_count=%d\n", traced_edges.num_events);

        corridormap::Walkable_Space_Build_Params params;
        params.bounds = obstacle_bounds;
        params.obstacles = &obstacles;
        params.obstacle_normals = &normals;
        params.features = &features;
        params.traced_edges = &traced_edges;
        params.spans = &edge_spans;
        params.edge_grid = &edge_csr;
        params.vertex_grid = &vert_csr;

        space = corridormap::create_walkable_space(&mem, features.num_vert_points, traced_edges.num_edges, traced_edges.num_events);
        corridormap::build_walkable_space(params, space);
    }

    {
        int path_verts[] = { 0, 2, 4, 8, 9, 13, 17, 21, 18, 26 };
        const int vert_path_size = sizeof(path_verts)/sizeof(path_verts[0]);
        corridormap::Vertex* vert_path[vert_path_size];

        for (int i = 0; i < vert_path_size; ++i)
        {
            vert_path[i] = space.vertices.items + path_verts[i];
        }

        corridormap::Half_Edge* edge_path[sizeof(path_verts)/sizeof(path_verts[0]) - 1];
        vertex_to_edge_path(space, vert_path, vert_path_size, edge_path);
        int edge_path_size = vert_path_size-1;
        int num_disks = corridormap::num_path_discs(space, edge_path, edge_path_size);
        corridor = create_corridor(&mem, num_disks, 5*num_disks);
        corridormap::extract(space, edge_path, edge_path_size, corridor);
        corridormap::shrink(corridor, 30.f);
        corridormap::triangulate(corridor, 8.f);
    }

    corridormap::Draw_State draw_state;
    draw_state.agent_radius = 15.f;
    draw_state.bounds_min = corridormap::make_vec2(obstacle_bounds.min);
    draw_state.bounds_max = corridormap::make_vec2(obstacle_bounds.max);
    draw_state.image_dimensions = corridormap::make_vec2(0, 0);
    draw_state.obstacles = &obstacles;
    draw_state.space = &space;
    draw_state.vg = vg;

    while (!glfwWindowShouldClose(window))
    {
        int window_width;
        int window_height;
        int framebuffer_width;
        int framebuffer_height;

        glfwGetWindowSize(window, &window_width, &window_height);
        glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
        float pxRatio = (float)framebuffer_width/(float)window_width;

        glViewport(0, 0, framebuffer_width, framebuffer_height);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);

        nvgBeginFrame(vg, window_width, window_height, pxRatio);

        float sx = (float)window_width/render_target_width;
        float sy = (float)window_height/render_target_height;
        float s = sx > sy ? sx : sy;
        nvgScale(vg, s, s);

        draw_state.image_dimensions = corridormap::make_vec2(float(screen_width), float(screen_height));
        corridormap::draw_walkable_space(draw_state);
        corridormap::draw_corridor(draw_state, corridor);
        corridormap::draw_portals(draw_state, corridor);
        corridormap::draw_continuous_path(draw_state, corridor, &mem, corridor.origin[0], corridor.origin[corridor.num_disks-1]);

        nvgEndFrame(vg);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    corridormap::deallocate(&mem, mesh);
    nvgDeleteGL3(vg);

    return 0;
}
