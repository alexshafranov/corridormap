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

#include <string.h>
#include "corridormap/memory.h"
#include "corridormap/build_alloc.h"

namespace corridormap {

Distance_Mesh allocate_distance_mesh(Memory* mem, int num_obstacle_polys, int max_verts)
{
    Distance_Mesh result;
    memset(&result, 0, sizeof(result));

    result.num_segments = 0;
    result.num_verts = 0;
    result.verts = allocate<Render_Vertex>(mem, max_verts);
    // 4 segments for border and num_obstacle_polys segments for obstacles + 1 segement for areas inside obstacles.
    result.num_segment_verts = allocate<int>(mem, 1 + num_border_segments + num_obstacle_polys);
    result.segment_colors = allocate<unsigned int>(mem, 1 + num_border_segments + num_obstacle_polys);
    return result;
}

void deallocate(Memory* mem, Distance_Mesh& mesh)
{
    mem->deallocate(mesh.verts);
    mem->deallocate(mesh.num_segment_verts);
    mem->deallocate(mesh.segment_colors);
    memset(&mesh, 0, sizeof(mesh));
}

Voronoi_Features allocate_voronoi_features(Memory* mem, int grid_width, int grid_height, int num_vert_points, int num_edge_points)
{
    Voronoi_Features result;
    memset(&result, 0, sizeof(result));

    result.grid_width = grid_width;
    result.grid_height = grid_height;
    result.num_vert_points = num_vert_points;
    result.num_edge_points = num_edge_points;
    result.verts = allocate<unsigned int>(mem, num_vert_points);
    result.edges = allocate<unsigned int>(mem, num_edge_points);
    result.vert_obstacle_ids = allocate<unsigned int>(mem, 4*num_vert_points);
    result.edge_obstacle_ids_1 = allocate<unsigned int>(mem, num_edge_points);
    result.edge_obstacle_ids_2 = allocate<unsigned int>(mem, num_edge_points);

    return result;
}

void deallocate(Memory* mem, Voronoi_Features& features)
{
    mem->deallocate(features.verts);
    mem->deallocate(features.edges);
    mem->deallocate(features.vert_obstacle_ids);
    mem->deallocate(features.edge_obstacle_ids_1);
    mem->deallocate(features.edge_obstacle_ids_2);
    memset(&features, 0, sizeof(features));
}

Footprint_Normals allocate_foorprint_normals(Memory* mem, int num_polygons, int num_poly_verts)
{
    Footprint_Normals result;
    memset(&result, 0, sizeof(result));

    result.num_obstacles = num_polygons + num_border_segments;
    result.num_normals = num_poly_verts + num_border_segments;
    result.x = allocate<float>(mem, result.num_normals);
    result.y = allocate<float>(mem, result.num_normals);
    result.num_obstacle_normals = allocate<int>(mem, result.num_obstacles);
    result.obstacle_normal_offsets = allocate<int>(mem, result.num_obstacles);

    return result;
}

// deallocates footprint normals. 'mem' must be the same that was used for allocation.
void deallocate(Memory* mem, Footprint_Normals& normals)
{
    mem->deallocate(normals.x);
    mem->deallocate(normals.y);
    mem->deallocate(normals.num_obstacle_normals);
    mem->deallocate(normals.obstacle_normal_offsets);
    memset(&normals, 0, sizeof(normals));
}

Voronoi_Edge_Normals allocate_voronoi_edge_normals(Memory* mem, int num_edge_points)
{
    Voronoi_Edge_Normals result;
    memset(&result, 0, sizeof(result));
    result.indices_1 = allocate<int>(mem, num_edge_points);
    result.indices_2 = allocate<int>(mem, num_edge_points);
    return result;
}

void deallocate(Memory* mem, Voronoi_Edge_Normals& result)
{
    mem->deallocate(result.indices_1);
    mem->deallocate(result.indices_2);
    memset(&result, 0, sizeof(result));
}

CSR_Grid allocate_csr_grid(Memory* mem, int num_rows, int num_cols, int num_non_zero)
{
    CSR_Grid result;
    memset(&result, 0, sizeof(result));

    result.num_rows = num_rows;
    result.num_cols = num_cols;
    result.num_nz = num_non_zero;
    result.column = allocate<int>(mem, num_non_zero);
    result.row_offset = allocate<int>(mem, num_rows + 1);

    return result;
}

void deallocate(Memory* mem, CSR_Grid& grid)
{
    mem->deallocate(grid.column);
    mem->deallocate(grid.row_offset);
    memset(&grid, 0, sizeof(grid));
}

Voronoi_Traced_Edges allocate_voronoi_traced_edges(Memory* mem, int num_voronoi_verts, int num_footprint_verts)
{
    Voronoi_Traced_Edges result;
    memset(&result, 0, sizeof(result));

    int max_edges = num_voronoi_verts*max_grid_neis/2;
    int max_events = num_footprint_verts*2;
    result.u = allocate<int>(mem, max_edges);
    result.v = allocate<int>(mem, max_edges);
    result.obstacle_ids_1 = allocate<unsigned int>(mem, max_edges);
    result.obstacle_ids_2 = allocate<unsigned int>(mem, max_edges);
    result.edge_event_offset = allocate<int>(mem, max_edges);
    result.edge_num_events = allocate<int>(mem, max_edges);
    result.events = allocate<int>(mem, max_events);

    return result;
}

void deallocate(Memory* mem, Voronoi_Traced_Edges& edges)
{
    mem->deallocate(edges.u);
    mem->deallocate(edges.v);
    memset(&edges, 0, sizeof(edges));
}

}
