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

/// data allocation functions.

#ifndef CORRIDORMAP_BUILD_ALLOC_H_
#define CORRIDORMAP_BUILD_ALLOC_H_

#include "corridormap/build_types.h"

namespace corridormap { class memory; }

namespace corridormap {

distance_mesh allocate_distance_mesh(memory* mem, int num_obstacle_polys, int max_verts);
voronoi_features allocate_voronoi_features(memory* mem, int grid_width, int grid_height, int num_vert_points, int num_edge_points);
footprint_normals allocate_foorprint_normals(memory* mem, int num_polygons, int num_poly_verts);
voronoi_edge_normals allocate_voronoi_edge_normals(memory* mem, int num_edge_points);
csr_grid allocate_csr_grid(memory* mem, int num_rows, int num_cols, int num_non_zero);
voronoi_traced_edges allocate_voronoi_traced_edges(memory* mem, int max_edges);

void deallocate(memory* mem, distance_mesh& mesh);
void deallocate(memory* mem, voronoi_features& features);
void deallocate(memory* mem, footprint_normals& normals);
void deallocate(memory* mem, voronoi_edge_normals& normals);
void deallocate(memory* mem, csr_grid& grid);
void deallocate(memory* mem, voronoi_traced_edges& edges);

}

#endif
