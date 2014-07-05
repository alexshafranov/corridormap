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

namespace corridormap { class Memory; }

namespace corridormap {

Distance_Mesh allocate_distance_mesh(Memory* mem, int num_obstacle_polys, int max_verts);
Voronoi_Features allocate_voronoi_features(Memory* mem, int grid_width, int grid_height, int num_vert_points, int num_edge_points);
Footprint_Normals allocate_foorprint_normals(Memory* mem, int num_polygons, int num_poly_verts);
Voronoi_Edge_Spans allocate_voronoi_edge_normals(Memory* mem, int num_edge_points);
CSR_Grid allocate_csr_grid(Memory* mem, int num_rows, int num_cols, int num_non_zero);
Voronoi_Traced_Edges allocate_voronoi_traced_edges(Memory* mem, int num_voronoi_verts, int num_footprint_verts);

void deallocate(Memory* mem, Distance_Mesh& mesh);
void deallocate(Memory* mem, Voronoi_Features& features);
void deallocate(Memory* mem, Footprint_Normals& normals);
void deallocate(Memory* mem, Voronoi_Edge_Spans& normals);
void deallocate(Memory* mem, CSR_Grid& grid);
void deallocate(Memory* mem, Voronoi_Traced_Edges& edges);

}

#endif
