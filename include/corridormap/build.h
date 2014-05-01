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

#ifndef CORRIDORMAP_BUILD_H_
#define CORRIDORMAP_BUILD_H_

#include "corridormap/types.h"
#include "corridormap/render_interface.h"
#include "corridormap/memory.h"

namespace corridormap {

// computes 2d bounding box of the input footprint.
bbox2 bounds(const footprint& f, float border);

// maximum distance for points and lines.
// computed such that distance mesh "covers" the full render target in ortho projection.
float max_distance(bbox2 scene_bbox);

// computes required number of triangles to represent a distance mesh for vertex (cone).
int vertex_distance_mesh_tris(float max_dist, float max_error);

// computes upper bound on number of vertices required for distance mesh.
int max_distance_mesh_verts(const footprint& f, float max_dist, float max_error);

// allocates distanece mesh data arrays.
distance_mesh allocate_distance_mesh(memory* mem, const footprint& f, float max_dist, float max_error);

// deallocates distance mesh data arrays. 'mem' must be the same that was used for allocation.
void deallocate_distance_mesh(memory* mem, distance_mesh& mesh);

// build distance mesh for the input footprint. polygon vertex becomes a cone sector, edge - a "tent".
void build_distance_mesh(const footprint& in, bbox2 bbox, float max_dist, float max_error, distance_mesh& out);

// renders distance mesh using the specified render interface.
void render_distance_mesh(renderer* render_iface, const distance_mesh& mesh);

// sets color of a segment to colors[segment_index % ncolors].
void set_segment_colors(distance_mesh& mesh, unsigned int* colors, int ncolors);

// inits library opencl runtime from render interface shared context, creates opencl command queue.
opencl_runtime init_opencl_runtime(const renderer::opencl_shared& shared);

// releases opencl objects.
void term_opencl_runtime(opencl_runtime& runtime);

// returns source code for the kernel with specified id.
const char* get_kernel_source(kernel_id id);

// creates and compiles library's opencl kernels.
compilation_status build_kernels(opencl_runtime& runtime);

// marks voronoi vertices and egdes in runtime.voronoi_vertices_img and voronoi_edges_img from voronoi_image.
cl_int mark_voronoi_features(opencl_runtime& runtime, cl_mem voronoi_image);

// draw marks back to original voronoi image.
cl_int debug_voronoi_features(opencl_runtime& runtime, cl_mem voronoi_image, cl_mem marks_image, unsigned int color, unsigned int border);

// compact voronoi features on gpu, storing results in runtime.voronoi_vertices_compacted_buf and runtime.voronoi_edges_compacted_buf buffers.
cl_int compact_voronoi_features(opencl_runtime& runtime);

}

#endif
