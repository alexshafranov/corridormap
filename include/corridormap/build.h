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
#include "corridormap/runtime_types.h"

namespace corridormap { class Memory; }
namespace corridormap { class Renderer; }

namespace corridormap {

// computes 2d bounding box of the input footprint.
Bbox2 bounds(const Footprint& f, float border);

// maximum distance for points and lines.
// computed such that distance mesh "covers" the full render target in ortho projection.
float max_distance(Bbox2 bounds);

// computes required number of triangles to represent a distance mesh for point (cone).
int distance_mesh_tris_for_point(float max_dist, float max_error);

// computes an upper bound on number of vertices required for distance mesh.
int max_distance_mesh_verts(const Footprint& f, float max_dist, float max_error);

// build distance mesh for the input footprint. polygon vertex becomes a cone sector, edge - a "tent".
void build_distance_mesh(const Footprint& in, Bbox2 bounds, float max_dist, float max_error, Distance_Mesh& out);

// renders distance mesh using the specified render interface.
void render_distance_mesh(Renderer* render_iface, const Distance_Mesh& mesh);

// debug: sets color of a segment to colors[segment_index % ncolors].
void set_segment_colors(Distance_Mesh& mesh, unsigned int* colors, int ncolors);

// go over all edges in the input footprint and compute normals for each.
void build_footprint_normals(const Footprint& in, Bbox2 bounds, Footprint_Normals& out);

// if edge point lies in vector space spanned by two consecutive normals assign first normal's index. otherwise keep zero.
void build_edge_point_normal_indices(const Voronoi_Features& features, const Footprint& obstacles,
                                     const Footprint_Normals& normals, Bbox2 bounds, Voronoi_Edge_Normals& out);

// build CSR (Compressed Sparse Row) grid representation from row-major list of non-zero element coordinates.
void build_csr(const unsigned int* nz_coords, CSR_Grid& out);

// index in array of non-zero cells or grid.num_nz if the cell is zero.
int nz(const CSR_Grid& grid, int row, int col);

// index in array of non-zero cells or grid.num_nz if the cell is zero. linear_index = row * num_cols + col.
int nz(const CSR_Grid& grid, int linear_index);

// finds all neighbours of 4-connected CSR grid cell.
CSR_Grid_Neis cell_neis(const CSR_Grid& grid, int row, int col);

// finds all neighbours of 4-connected CSR grid cell. linear_index = row * num_cols + col.
CSR_Grid_Neis cell_neis(const CSR_Grid& grid, int linear_index);

// go over pixels to find connections between voronoi vertices and event points - the points where closest obstacles change.
// also arrange edge obstacle ids in edge_normal_indices and Voronoi_Features such that if two edge points a and b are on the same edge,
// side_1[a] == side_1[b] and side_2[a] == side_2[b].
void trace_edges(Memory* scratch, const CSR_Grid& vertices, const CSR_Grid& edges,
                 Voronoi_Edge_Normals& edge_normal_indices, Voronoi_Features& features, Voronoi_Traced_Edges& out);

// builds annotated voronoi diagram (i.e. Explicit Corridor Map) represented as a half-edge mesh.
void build_walkable_space(const Footprint& obstacles, const int* obstacle_offsets, Bbox2 bounds, const Voronoi_Features& features,
                          const CSR_Grid& edge_grid,  const CSR_Grid& vertex_grid, const Voronoi_Traced_Edges& traced_edges, Walkable_Space& out);

}

#endif
