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

#ifdef CORRIDORMAP_CONFIG_USE_CLEW
#include <clew.h>
#endif

#include <string.h>

#include "corridormap/memory.h"
#include "corridormap/build_ocl.h"

#define CORRIDORMAP_CHECK_OCL(error_code)   \
    if (error_code != CL_SUCCESS)           \
    {                                       \
        return error_code;                  \
    }                                       \

namespace corridormap {

opencl_runtime init_opencl_runtime(const renderer::opencl_shared& shared)
{
    opencl_runtime runtime;
    memset(&runtime, 0, sizeof(opencl_runtime));

    cl_int error_code;
    runtime.queue = clCreateCommandQueue(shared.context, shared.device, 0, &error_code);

    if (error_code != CL_SUCCESS)
    {
        return runtime;
    }

    runtime.context = shared.context;
    runtime.device = shared.device;

    return runtime;
}

void term_opencl_runtime(opencl_runtime& runtime)
{
    clReleaseMemObject(runtime.voronoi_vertices_img);
    clReleaseMemObject(runtime.voronoi_edges_img);
    clReleaseMemObject(runtime.voronoi_vertices_compacted_buf);
    clReleaseMemObject(runtime.voronoi_edges_compacted_buf);
    clReleaseMemObject(runtime.compaction_sums_buf);
    clReleaseMemObject(runtime.compaction_offsets_buf);

    clReleaseCommandQueue(runtime.queue);

    for (int i = 0; i < kernel_id_count; ++i)
    {
        clReleaseKernel(runtime.kernels[i]);
    }
}

#define CORRIDORMAP_KERNEL_ID(NAME) extern const char* kernel_##NAME##_source;
#include "corridormap/kernel_id.inl"
#undef CORRIDORMAP_KERNEL_ID

namespace
{
    const char* kernel_source[] =
    {
        #define CORRIDORMAP_KERNEL_ID(NAME) kernel_##NAME##_source,
        #include "corridormap/kernel_id.inl"
        #undef CORRIDORMAP_KERNEL_ID
        0,
    };
}

const char* get_kernel_source(kernel_id id)
{
    return kernel_source[id];
}

compilation_status build_kernels(opencl_runtime& runtime)
{
    compilation_status status;
    status.code = CL_SUCCESS;
    status.kernel = kernel_id_count;

    for (int i = 0; i < kernel_id_count; ++i)
    {
        status.kernel = static_cast<kernel_id>(i);

        runtime.programs[i] = clCreateProgramWithSource(runtime.context, 1, &kernel_source[i], 0, &status.code);

        if (status.code != CL_SUCCESS)
        {
            return status;
        }

        status.code = clBuildProgram(runtime.programs[i], 1, &runtime.device, 0, 0, 0);

        if (status.code != CL_SUCCESS)
        {
            return status;
        }

        runtime.kernels[i] = clCreateKernel(runtime.programs[i], "run", &status.code);

        if (status.code != CL_SUCCESS)
        {
            return status;
        }
    }

    status.kernel = kernel_id_count;
    return status;
}

namespace
{
    cl_int allocate_voronoi_features(opencl_runtime& runtime, cl_mem voronoi_image)
    {
        cl_int error_code;

        size_t width;
        clGetImageInfo(voronoi_image, CL_IMAGE_WIDTH, sizeof(width), &width, 0);

        size_t height;
        clGetImageInfo(voronoi_image, CL_IMAGE_HEIGHT, sizeof(height), &height, 0);

        // RGBA8 must be supported by all implementations according to OpenCL 1.1 specification.
        cl_image_format format;
        format.image_channel_order = CL_RGBA;
        format.image_channel_data_type = CL_UNSIGNED_INT8;

        runtime.voronoi_vertices_img = clCreateImage2D(runtime.context, CL_MEM_READ_WRITE, &format, width, height, 0, 0, &error_code);
        CORRIDORMAP_CHECK_OCL(error_code);

        runtime.voronoi_edges_img = clCreateImage2D(runtime.context, CL_MEM_READ_WRITE, &format, width, height, 0, 0, &error_code);
        CORRIDORMAP_CHECK_OCL(error_code);

        return CL_SUCCESS;
    }
}

cl_int mark_voronoi_features(opencl_runtime& runtime, cl_mem voronoi_image)
{
    cl_int error_code = allocate_voronoi_features(runtime, voronoi_image);
    CORRIDORMAP_CHECK_OCL(error_code);

    size_t width;
    clGetImageInfo(voronoi_image, CL_IMAGE_WIDTH, sizeof(width), &width, 0);

    size_t height;
    clGetImageInfo(voronoi_image, CL_IMAGE_HEIGHT, sizeof(height), &height, 0);

    size_t global_work_size[] = { width, height };

    cl_kernel kernel = runtime.kernels[kernel_id_mark_features];

    clSetKernelArg(kernel, 0, sizeof(cl_mem), &voronoi_image);
    clSetKernelArg(kernel, 1, sizeof(cl_mem), &runtime.voronoi_vertices_img);
    clSetKernelArg(kernel, 2, sizeof(cl_mem), &runtime.voronoi_edges_img);

    return clEnqueueNDRangeKernel(runtime.queue, kernel, 2, 0, global_work_size, 0, 0, 0, 0);
}

cl_int debug_voronoi_features(opencl_runtime& runtime, cl_mem voronoi_image, cl_mem marks_image, unsigned int color, unsigned int border)
{
    size_t width;
    clGetImageInfo(voronoi_image, CL_IMAGE_WIDTH, sizeof(width), &width, 0);

    size_t height;
    clGetImageInfo(voronoi_image, CL_IMAGE_HEIGHT, sizeof(height), &height, 0);

    size_t global_work_size[] = { width, height };

    cl_kernel kernel = runtime.kernels[kernel_id_mark_features_debug];

    cl_uint color_value = static_cast<cl_uint>(color);
    cl_int border_value = static_cast<cl_int>(border);
    cl_uint width_value = static_cast<cl_uint>(width);
    cl_uint height_value = static_cast<cl_uint>(height);

    clSetKernelArg(kernel, 0, sizeof(cl_mem), &marks_image);
    clSetKernelArg(kernel, 1, sizeof(cl_mem), &voronoi_image);
    clSetKernelArg(kernel, 2, sizeof(cl_uint), &color_value);
    clSetKernelArg(kernel, 3, sizeof(cl_int), &border_value);
    clSetKernelArg(kernel, 4, sizeof(cl_uint), &width_value);
    clSetKernelArg(kernel, 5, sizeof(cl_uint), &height_value);

    return clEnqueueNDRangeKernel(runtime.queue, kernel, 2, 0, global_work_size, 0, 0, 0, 0);
}

namespace
{
    size_t get_compaction_wgsize(opencl_runtime& runtime)
    {
        cl_kernel kernel_reduce = runtime.kernels[kernel_id_compaction_reduce];

        size_t max_wgsize = 0;
        clGetKernelWorkGroupInfo(kernel_reduce, runtime.device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &max_wgsize, 0);

        size_t wg_size = (max_wgsize >= 128) ? 128 : max_wgsize;

        return wg_size;
    }
}

namespace
{
    cl_int compact_features(opencl_runtime& runtime, cl_mem image, cl_mem sums_buf, cl_mem offsets_buf, cl_mem* compacted_buf, cl_uint* count)
    {
        cl_int error_code;

        size_t width;
        clGetImageInfo(image, CL_IMAGE_WIDTH, sizeof(width), &width, 0);

        size_t height;
        clGetImageInfo(image, CL_IMAGE_HEIGHT, sizeof(height), &height, 0);

        cl_uint pixel_count = static_cast<cl_uint>(width * height);

        cl_kernel kernel_reduce = runtime.kernels[kernel_id_compaction_reduce];
        cl_kernel kernel_scan = runtime.kernels[kernel_id_compaction_scan_partials];
        cl_kernel kernel_output = runtime.kernels[kernel_id_compaction_output];

        size_t wg_size = get_compaction_wgsize(runtime);

        const size_t simd_size = 32;
        const size_t local_mem_size = 2*wg_size*sizeof(cl_uint);

        cl_event event_reduce;
        cl_event event_scan;
        cl_event event_read;
        cl_event event_output;

        // 1. reduce.

        clSetKernelArg(kernel_reduce, 0, sizeof(cl_mem), &image);
        clSetKernelArg(kernel_reduce, 1, sizeof(cl_mem), &sums_buf);
        clSetKernelArg(kernel_reduce, 2, local_mem_size, 0);
        clSetKernelArg(kernel_reduce, 3, sizeof(cl_uint), &pixel_count);

        size_t reduce_global_work_size = 2*wg_size*simd_size;
        error_code = clEnqueueNDRangeKernel(runtime.queue, kernel_reduce, 1, 0, &reduce_global_work_size, &wg_size, 0, 0, &event_reduce);
        CORRIDORMAP_CHECK_OCL(error_code);

        // 2. scan partials.

        clSetKernelArg(kernel_scan, 0, sizeof(cl_mem), &sums_buf);
        clSetKernelArg(kernel_scan, 1, sizeof(cl_mem), &offsets_buf);
        clSetKernelArg(kernel_scan, 2, local_mem_size, 0);

        size_t scan_global_work_size = wg_size;
        error_code = clEnqueueNDRangeKernel(runtime.queue, kernel_scan, 1, 0, &scan_global_work_size, &wg_size, 1, &event_reduce, &event_scan);
        CORRIDORMAP_CHECK_OCL(error_code);

        const size_t result_offset = 2*wg_size*sizeof(cl_uint);
        error_code = clEnqueueReadBuffer(runtime.queue, offsets_buf, CL_FALSE, result_offset, sizeof(cl_uint), count, 1, &event_scan, &event_read);
        CORRIDORMAP_CHECK_OCL(error_code);

        clWaitForEvents(1, &event_read);

        if (*count == 0)
        {
            compacted_buf = 0;
            return error_code;
        }

        *compacted_buf = clCreateBuffer(runtime.context, CL_MEM_READ_WRITE, (*count)*sizeof(cl_uint), 0, &error_code);

        // 3. compacted output.

        clSetKernelArg(kernel_output, 0, sizeof(cl_mem), &image);
        clSetKernelArg(kernel_output, 1, sizeof(cl_mem), compacted_buf);
        clSetKernelArg(kernel_output, 2, sizeof(cl_mem), &sums_buf);
        clSetKernelArg(kernel_output, 3, sizeof(cl_mem), &offsets_buf);
        clSetKernelArg(kernel_output, 4, local_mem_size, 0);
        clSetKernelArg(kernel_output, 5, sizeof(cl_uint), &pixel_count);

        size_t output_global_work_size = 2*wg_size*simd_size;
        error_code = clEnqueueNDRangeKernel(runtime.queue, kernel_output, 1, 0, &output_global_work_size, &wg_size, 0, 0, &event_output);
        CORRIDORMAP_CHECK_OCL(error_code);

        clWaitForEvents(1, &event_output);

        return error_code;
    }
}

cl_int compact_voronoi_features(opencl_runtime& runtime)
{
    cl_int error_code;

    size_t wg_size = get_compaction_wgsize(runtime);

    runtime.compaction_sums_buf = clCreateBuffer(runtime.context, CL_MEM_READ_WRITE, 2*wg_size*sizeof(cl_uint), 0, &error_code);
    CORRIDORMAP_CHECK_OCL(error_code);
    runtime.compaction_offsets_buf = clCreateBuffer(runtime.context, CL_MEM_READ_WRITE, (1 + 2*wg_size) * sizeof(cl_uint), 0, &error_code);
    CORRIDORMAP_CHECK_OCL(error_code);

    error_code = compact_features(runtime, runtime.voronoi_vertices_img, runtime.compaction_sums_buf, runtime.compaction_offsets_buf, &runtime.voronoi_vertices_compacted_buf, &runtime.voronoi_vertex_mark_count);
    CORRIDORMAP_CHECK_OCL(error_code);

    error_code = compact_features(runtime, runtime.voronoi_edges_img, runtime.compaction_sums_buf, runtime.compaction_offsets_buf, &runtime.voronoi_edges_compacted_buf, &runtime.voronoi_edge_mark_count);
    CORRIDORMAP_CHECK_OCL(error_code);

    return error_code;
}

cl_int store_obstacle_ids(opencl_runtime& runtime, cl_mem voronoi_image)
{
    cl_int error_code;

    runtime.voronoi_edge_ids_left = clCreateBuffer(runtime.context, CL_MEM_WRITE_ONLY, runtime.voronoi_edge_mark_count*sizeof(cl_uint), 0, &error_code);
    CORRIDORMAP_CHECK_OCL(error_code);
    runtime.voronoi_edge_ids_right = clCreateBuffer(runtime.context, CL_MEM_WRITE_ONLY, runtime.voronoi_edge_mark_count*sizeof(cl_uint), 0, &error_code);
    CORRIDORMAP_CHECK_OCL(error_code);
    runtime.voronoi_vertex_ids = clCreateBuffer(runtime.context, CL_MEM_WRITE_ONLY, 4*runtime.voronoi_vertex_mark_count*sizeof(cl_uint), 0, &error_code);
    CORRIDORMAP_CHECK_OCL(error_code);

    cl_kernel kernel_edge = runtime.kernels[kernel_id_store_edge_obstacle_ids];
    {
        clSetKernelArg(kernel_edge, 0, sizeof(cl_mem), &voronoi_image);
        clSetKernelArg(kernel_edge, 1, sizeof(cl_mem), &runtime.voronoi_edges_compacted_buf);
        clSetKernelArg(kernel_edge, 2, sizeof(cl_mem), &runtime.voronoi_edge_ids_left);
        clSetKernelArg(kernel_edge, 3, sizeof(cl_mem), &runtime.voronoi_edge_ids_right);

        size_t global_work_size = runtime.voronoi_edge_mark_count;
        error_code = clEnqueueNDRangeKernel(runtime.queue, kernel_edge, 1, 0, &global_work_size, 0, 0, 0, 0);
        CORRIDORMAP_CHECK_OCL(error_code);
    }

    cl_kernel kernel_vertex = runtime.kernels[kernel_id_store_vertex_obstacle_ids];
    {
        clSetKernelArg(kernel_vertex, 0, sizeof(cl_mem), &voronoi_image);
        clSetKernelArg(kernel_vertex, 1, sizeof(cl_mem), &runtime.voronoi_vertices_compacted_buf);
        clSetKernelArg(kernel_vertex, 2, sizeof(cl_mem), &runtime.voronoi_vertex_ids);

        size_t global_work_size = runtime.voronoi_vertex_mark_count;
        error_code = clEnqueueNDRangeKernel(runtime.queue, kernel_vertex, 1, 0, &global_work_size, 0, 0, 0, 0);
        CORRIDORMAP_CHECK_OCL(error_code);
    }

    return error_code;
}

cl_int transfer_voronoi_features(opencl_runtime& runtime, voronoi_features& features)
{
    cl_int error_code;

    cl_event events[5];

    error_code = clEnqueueReadBuffer(runtime.queue, runtime.voronoi_vertices_compacted_buf, CL_FALSE, 0, runtime.voronoi_vertex_mark_count*sizeof(cl_uint), features.verts, 0, 0, &events[0]);
    CORRIDORMAP_CHECK_OCL(error_code);
    error_code = clEnqueueReadBuffer(runtime.queue, runtime.voronoi_edges_compacted_buf, CL_FALSE, 0, runtime.voronoi_edge_mark_count*sizeof(cl_uint), features.edges, 0, 0, &events[1]);
    CORRIDORMAP_CHECK_OCL(error_code);
    error_code = clEnqueueReadBuffer(runtime.queue, runtime.voronoi_vertex_ids, CL_FALSE, 0, 4*runtime.voronoi_vertex_mark_count*sizeof(cl_uint), features.vert_obstacle_ids, 0, 0, &events[2]);
    CORRIDORMAP_CHECK_OCL(error_code);
    error_code = clEnqueueReadBuffer(runtime.queue, runtime.voronoi_edge_ids_left, CL_FALSE, 0, runtime.voronoi_edge_mark_count*sizeof(cl_uint), features.edge_obstacle_ids_left, 0, 0, &events[3]);
    CORRIDORMAP_CHECK_OCL(error_code);
    error_code = clEnqueueReadBuffer(runtime.queue, runtime.voronoi_edge_ids_right, CL_FALSE, 0, runtime.voronoi_edge_mark_count*sizeof(cl_uint), features.edge_obstacle_ids_right, 0, 0, &events[4]);
    CORRIDORMAP_CHECK_OCL(error_code);

    clWaitForEvents(sizeof(events)/sizeof(events[0]), events);

    return error_code;
}

}
