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

distance_mesh allocate_distance_mesh(memory* mem, int num_obstacle_polys, int max_verts)
{
    distance_mesh result;
    memset(&result, 0, sizeof(result));

    result.num_segments = 0;
    result.num_verts = 0;
    result.verts = allocate<render_vertex>(mem, max_verts);
    // 4 segments for border and num_obstacle_polys segments for obstacles.
    result.num_segment_verts = allocate<int>(mem, num_border_segments + num_obstacle_polys);
    result.segment_colors = allocate<unsigned int>(mem, num_border_segments + num_obstacle_polys);
    return result;
}

void deallocate(memory* mem, distance_mesh& mesh)
{
    mem->deallocate(mesh.verts);
    mem->deallocate(mesh.num_segment_verts);
    mem->deallocate(mesh.segment_colors);
    memset(&mesh, 0, sizeof(mesh));
}

voronoi_features allocate_voronoi_features(memory* mem, int grid_width, int grid_height, int num_vert_points, int num_edge_points)
{
    voronoi_features result;
    memset(&result, 0, sizeof(result));

    result.grid_width = grid_width;
    result.grid_height = grid_height;
    result.num_vert_points = num_vert_points;
    result.num_edge_points = num_edge_points;
    result.verts = allocate<unsigned int>(mem, num_vert_points);
    result.edges = allocate<unsigned int>(mem, num_edge_points);
    result.vert_obstacle_ids = allocate<unsigned int>(mem, 4*num_vert_points);
    result.edge_obstacle_ids_left = allocate<unsigned int>(mem, num_edge_points);
    result.edge_obstacle_ids_right = allocate<unsigned int>(mem, num_edge_points);

    return result;
}

void deallocate(memory* mem, voronoi_features& features)
{
    mem->deallocate(features.verts);
    mem->deallocate(features.edges);
    mem->deallocate(features.vert_obstacle_ids);
    mem->deallocate(features.edge_obstacle_ids_left);
    mem->deallocate(features.edge_obstacle_ids_right);
    memset(&features, 0, sizeof(features));
}

footprint_normals allocate_foorprint_normals(memory* mem, int num_polygons, int num_poly_verts)
{
    footprint_normals result;
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
void deallocate(memory* mem, footprint_normals& normals)
{
    mem->deallocate(normals.x);
    mem->deallocate(normals.y);
    mem->deallocate(normals.num_obstacle_normals);
    mem->deallocate(normals.obstacle_normal_offsets);
    memset(&normals, 0, sizeof(normals));
}

voronoi_edge_normals allocate_voronoi_edge_normals(memory* mem, int num_edge_points)
{
    voronoi_edge_normals result;
    memset(&result, 0, sizeof(result));
    result.edge_normal_indices_left = allocate<int>(mem, num_edge_points);
    result.edge_normal_indices_right = allocate<int>(mem, num_edge_points);
    return result;
}

void deallocate(memory* mem, voronoi_edge_normals& result)
{
    mem->deallocate(result.edge_normal_indices_left);
    mem->deallocate(result.edge_normal_indices_right);
    memset(&result, 0, sizeof(result));
}

csr_grid allocate_csr_grid(memory* mem, int num_rows, int num_cols, int num_non_zero)
{
    csr_grid result;
    memset(&result, 0, sizeof(result));

    result.num_rows = num_rows;
    result.num_cols = num_cols;
    result.num_nz = num_non_zero;
    result.column = allocate<int>(mem, num_non_zero);
    result.row_offset = allocate<int>(mem, num_rows + 1);

    return result;
}

void deallocate(memory* mem, csr_grid& grid)
{
    mem->deallocate(grid.column);
    mem->deallocate(grid.row_offset);
    memset(&grid, 0, sizeof(grid));
}

voronoi_traced_edges allocate_voronoi_traced_edges(memory* mem, int num_verts)
{
    voronoi_traced_edges result;
    memset(&result, 0, sizeof(result));

    result.max_edges = num_verts*max_grid_neis/2;
    result.u = allocate<int>(mem, result.max_edges);
    result.v = allocate<int>(mem, result.max_edges);

    return result;
}

void deallocate(memory* mem, voronoi_traced_edges& edges)
{
    mem->deallocate(edges.u);
    mem->deallocate(edges.v);
    memset(&edges, 0, sizeof(edges));
}

}
