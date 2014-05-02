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

#ifndef CORRIDORMAP_TYPES_H_
#define CORRIDORMAP_TYPES_H_

#include <CL/opencl.h>

namespace corridormap {

// 2d bounding box.
struct bbox2
{
    float min[2];
    float max[2];
};

// obstacles represented as a set of 2d convex polygons. polys are stored in CCW order.
struct footprint
{
    // the number of polygons.
    int num_polys;
    // the total number of vertices.
    int num_verts;
    // x coords indexed in [0..total_verts] range.
    float* x;
    // y coords indexed in [0..total_verts] range.
    float* y;
    // array of vertex counts per poly, indexed in [0..num_polys] range.
    int* num_poly_verts;
};

// 3d vertex used for distance_mesh.
struct vertex
{
    float x;
    float y;
    float z;
};

// segmented distance mesh suitable for rendering.
// each segment represents one footprint polygon.
// triangles are stored as list in ccw order.
struct distance_mesh
{
    // the number of segments.
    int num_segments;
    // the total number of vertices.
    int num_verts;
    // vertex array indexed in [0..num_verts] range.
    vertex* verts;
    // the number of vertices per segment. indexed in [0..num_segments] range.
    int* num_segment_verts;
    // segment colors. indexed in [0..num_segments] range.
    unsigned int* segment_colors;
};

// voronoi vertices and edges detected from the distance mesh render.
struct voronoi_features
{
    // number of voronoi vertex points.
    int num_vert_points;
    // number of voronoi edge points.
    int num_edge_points;
    // grid indices (y*width + x) of vertex points. [0..num_vert_points].
    unsigned int* verts;
    // grid indices (y*width + x) of edge points. [0..num_edge_points].
    unsigned int* edges;
    // IDs (colors) of obstacles surrounding each vertex. [0..4*num_vert_points].
    unsigned int* vert_obstacle_ids;
    // IDs (colors) of obstacles from the left side of each edge point. [0..num_edge_points].
    unsigned int* edge_obstacle_ids_left;
    // IDs (colors) of obstacles from the right side of each edge point. [0..num_edge_points].
    unsigned int* edge_obstacle_ids_right;
};

enum kernel_id
{
    #define CORRIDORMAP_KERNEL_ID(NAME) kernel_id_##NAME,
    #include "corridormap/kernel_id.inl"
    #undef CORRIDORMAP_KERNEL_ID
    kernel_id_count,
};

// holds opencl api objects used by the library.
struct opencl_runtime
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
    // stores left side obstacle id (color) for each edge point from voronoi_edges_compacted_buf.
    cl_mem voronoi_edge_ids_left;
    // stores right side obstacle id (color) for each edge point from voronoi_edges_compacted_buf.
    cl_mem voronoi_edge_ids_right;

    // temp buffer.
    cl_mem compaction_sums_buf;
    // temp buffer.
    cl_mem compaction_offsets_buf;

    // number of voronoi vertices.
    size_t voronoi_vertex_mark_count;
    // number of points forming voronoi edges.
    size_t voronoi_edge_mark_count;
};

// represents the result of corridormap::build_kernels.
struct compilation_status
{
    // opencl error code.
    cl_int    code;
    // kernel_id during which build the error happened or kernel_id_count on success.
    kernel_id kernel;
};

}

#endif
