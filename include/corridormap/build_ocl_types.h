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

#ifndef CORRIDORMAP_OCL_TYPES_H_
#define CORRIDORMAP_OCL_TYPES_H_

#include <CL/opencl.h>

namespace corridormap {

enum Kernel_Id
{
    #define CORRIDORMAP_KERNEL_ID(NAME) kernel_id_##NAME,
    #include "corridormap/kernel_id.inl"
    #undef CORRIDORMAP_KERNEL_ID
    kernel_id_count,
};

// Holds opencl api objects used by the library.
struct Opencl_Runtime
{
    // opencl context.
    cl_context context;
    // opencl command queue.
    cl_command_queue queue;
    // opencl device.
    cl_device_id device;
    // array of kernel objects used by the library.
    cl_kernel kernels[kernel_id_count];
    // array of kernel programs.
    cl_program programs[kernel_id_count];

    // 2d image with voronoi vertices marked.
    cl_mem voronoi_vertices_img;
    // 2d image with voronoi edges marked.
    cl_mem voronoi_edges_img;

    // compactly stores indices of non-zero elements in voronoi_vertices_img.
    cl_mem voronoi_vertices_compacted_buf;
    // compactly indices of non-zero elements in voronoi_edges_img.
    cl_mem voronoi_edges_compacted_buf;

    // stores four obstacle ids per each vertex index in voronoi_vertices_compacted_buf.
    cl_mem voronoi_vertex_ids;
    // stores one side obstacle id (color) for each edge point from voronoi_edges_compacted_buf.
    cl_mem voronoi_edge_ids_1;
    // stores one side obstacle id (color) for each edge point from voronoi_edges_compacted_buf.
    cl_mem voronoi_edge_ids_2;

    // temp buffer.
    cl_mem compaction_sums_buf;
    // temp buffer.
    cl_mem compaction_offsets_buf;

    // number of voronoi vertices.
    cl_uint voronoi_vertex_mark_count;
    // number of points forming voronoi edges.
    cl_uint voronoi_edge_mark_count;
};

// Represents the result of corridormap::build_kernels.
struct Compilation_Status
{
    // opencl error code.
    cl_int    code;
    // kernel_id during which build the error happened or kernel_id_count on success.
    Kernel_Id kernel;
};

}

#endif
