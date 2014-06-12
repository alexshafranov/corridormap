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

#include <math.h>

#include "corridormap/build_types.h"
#include "corridormap/runtime_types.h"

namespace corridormap { class memory; }
namespace corridormap { class renderer; }

namespace corridormap {

// computes 2d bounding box of the input footprint.
bbox2 bounds(const footprint& f, float border);

// maximum distance for points and lines.
// computed such that distance mesh "covers" the full render target in ortho projection.
float max_distance(bbox2 bounds);

// computes required number of triangles to represent a distance mesh for point (cone).
int distance_mesh_tris_for_point(float max_dist, float max_error);

// computes an upper bound on number of vertices required for distance mesh.
int max_distance_mesh_verts(const footprint& f, float max_dist, float max_error);

// build distance mesh for the input footprint. polygon vertex becomes a cone sector, edge - a "tent".
void build_distance_mesh(const footprint& in, bbox2 bounds, float max_dist, float max_error, distance_mesh& out);

// renders distance mesh using the specified render interface.
void render_distance_mesh(renderer* render_iface, const distance_mesh& mesh);

// debug: sets color of a segment to colors[segment_index % ncolors].
void set_segment_colors(distance_mesh& mesh, unsigned int* colors, int ncolors);

// go over all edges in the input footprint and compute normals for each.
void build_footprint_normals(const footprint& in, bbox2 bounds, footprint_normals& out);

// if edge point lies in vector space spanned by two consecutive normals assign first normal's index. otherwise keep zero.
void build_edge_point_normal_indices(const voronoi_features& features, const footprint& obstacles,
                                     const footprint_normals& normals, bbox2 bounds, voronoi_edge_normals& out);

// build CSR (Compressed Sparse Row) grid representation from row-major list of non-zero element coordinates.
void build_csr(const unsigned int* nz_coords, csr_grid& out);

// index in array of non-zero cells or grid.num_nz if the cell is zero.
int nz(const csr_grid& grid, int row, int col);

// index in array of non-zero cells or grid.num_nz if the cell is zero. linear_index = row * num_cols + col.
int nz(const csr_grid& grid, int linear_index);

// finds all neighbours of 4-connected CSR grid cell.
csr_grid_neis cell_neis(const csr_grid& grid, int row, int col);

// finds all neighbours of 4-connected CSR grid cell. linear_index = row * num_cols + col.
csr_grid_neis cell_neis(const csr_grid& grid, int linear_index);

// go over pixels to find connections between voronoi vertices and event points - the points where closest obstacles change.
// also arrange edge obstacle ids in edge_normal_indices and voronoi_features such that if two edge points a and b are on the same edge,
// side_1[a] == side_1[b] and side_2[a] == side_2[b].
void trace_edges(memory* scratch, const csr_grid& vertices, const csr_grid& edges,
                 voronoi_edge_normals& edge_normal_indices, voronoi_features& features, voronoi_traced_edges& out);

// builds annotated voronoi diagram (i.e. Explicit Corridor Map) represented as a half-edge mesh.
void build_voronoi_diagram(const footprint& obstacles, const int* obstacle_offsets, bbox2 bounds, const voronoi_features& features,
                           const csr_grid& edge_grid,  const csr_grid& vertex_grid, const voronoi_traced_edges& traced_edges, voronoi_diagram& out);

}

#endif
