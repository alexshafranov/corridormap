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

#include "corridormap/build_types.h"

namespace corridormap { class renderer; }

namespace corridormap {

// computes 2d bounding box of the input footprint.
bbox2 bounds(const footprint& f, float border);

// maximum distance for points and lines.
// computed such that distance mesh "covers" the full render target in ortho projection.
float max_distance(bbox2 scene_bbox);

// computes required number of triangles to represent a distance mesh for vertex (cone).
int vertex_distance_mesh_tris(float max_dist, float max_error);

// computes an upper bound on number of vertices required for distance mesh.
int max_distance_mesh_verts(const footprint& f, float max_dist, float max_error);

// build distance mesh for the input footprint. polygon vertex becomes a cone sector, edge - a "tent".
void build_distance_mesh(const footprint& in, bbox2 bbox, float max_dist, float max_error, distance_mesh& out);

// renders distance mesh using the specified render interface.
void render_distance_mesh(renderer* render_iface, const distance_mesh& mesh);

// debug: sets color of a segment to colors[segment_index % ncolors].
void set_segment_colors(distance_mesh& mesh, unsigned int* colors, int ncolors);

// go over all edges in the input footprint and compute normals for each.
void build_footprint_normals(const footprint& in, bbox2 bbox, footprint_normals& out);

// if edge point lies in vector space spanned by two consecutive normals assign first normal's index. otherwise keep zero.
void build_edge_point_normal_indices(const voronoi_features& features, const footprint& obstacles,
                                     const footprint_normals& normals, voronoi_edge_normals& out);

// for each position (pos_x, pos_y), obstacle index in obstacle_offsets and footprint (obstacle_ids)
// compute closest point on that obstacle and output it to out_x, out_y.
void compute_closest_points(const footprint& obstacles, const int* obstacle_offsets, const float* pos_x, const float* pos_y,
                            const unsigned int* obstacle_ids, const int num_points, float* out_x, float* out_y);

// build CSR (Compressed Sparse Row) grid representation from row-major list of non-zero element coordinates.
void build_csr(const unsigned int* nz_coords, csr_grid& out);

}

#endif
