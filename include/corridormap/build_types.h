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

#ifndef CORRIDORMAP_BUILD_TYPES_H_
#define CORRIDORMAP_BUILD_TYPES_H_

// 
// types used during construction of the corridor map.
//

namespace corridormap {

// obstacles represented as a set of 2d convex polygons. polys are stored in CCW order.
struct footprint
{
    // the number of polygons.
    int num_polys;
    // the total number of vertices.
    int num_verts;
    // x coords indexed in [0..num_verts) range.
    float* x;
    // y coords indexed in [0..num_verts) range.
    float* y;
    // array of vertex counts per poly, indexed in [0..num_polys) range.
    int* num_poly_verts;
};

// 2d bounding box.
struct bbox2
{
    float min[2];
    float max[2];
};

// 3d vertex used for distance_mesh.
struct render_vertex
{
    float x;
    float y;
    float z;
};

// segmented distance mesh suitable for rendering.
// each segment represents one footprint polygon.
// triangles are stored as a list of vertices in CCW order.
struct distance_mesh
{
    // the number of segments.
    int num_segments;
    // the total number of vertices.
    int num_verts;
    // vertex array indexed in [0..num_verts) range.
    render_vertex* verts;
    // the number of vertices per segment. indexed in [0..num_segments) range.
    int* num_segment_verts;
    // segment colors. indexed in [0..num_segments) range.
    unsigned int* segment_colors;
};

// voronoi vertices and edges detected from the distance mesh render.
struct voronoi_features
{
    // rasterization grid width.
    int grid_width;
    // number of voronoi vertex points.
    int num_vert_points;
    // number of voronoi edge points.
    int num_edge_points;
    // grid indices (y*grid_width + x) of vertex points. [0..num_vert_points).
    unsigned int* verts;
    // grid indices (y*grid_width + x) of edge points. [0..num_edge_points).
    unsigned int* edges;
    // IDs (colors) of obstacles surrounding each vertex. [0..4*num_vert_points).
    unsigned int* vert_obstacle_ids;
    // IDs (colors) of obstacles from the left side of each edge point. [0..num_edge_points).
    unsigned int* edge_obstacle_ids_left;
    // IDs (colors) of obstacles from the right side of each edge point. [0..num_edge_points).
    unsigned int* edge_obstacle_ids_right;
};

// obstacle polygon edge normals.
struct footprint_normals
{
    // number of obstacles in footprint (number of polys + 4 border segments).
    int num_obstacles;
    // total number of normals (one per each edge in footprint).
    int num_normals;
    // x coord indexed in [0..num_normals)
    float* x;
    // y coord indexed in [0..num_normals)
    float* y;
    // array of normal counts per obstacle, indexed in [0..num_obstacles) range.
    int* num_obstacle_normals;
    // offsets in x, y arrays for each poly, indexed in [0..num_obstacles) range.
    int* obstacle_normal_offsets;
};

// indices of obstacle normals associated with voronoi edge points.
struct voronoi_edge_normals
{
    // normal index on the left side.
    // equals i+1 if edge point lies in normals[i], normals[i+1] span. 0 otherwise.
    // [0..num_edge_points]
    int* edge_normal_indices_left;
    // normal index on the right side.
    // equals i+1 if edge point lies in normals[i], normals[i+1] span. 0 otherwise.
    // [0..num_edge_points]
    int* edge_normal_indices_right;
};

// Compressed-Sparse-Row format for boolean grid,
// used as a fast lookup of voronoi features during tracing.
struct csr_grid
{
    // number of rows in the grid.
    int num_rows;
    // number of columns in the grid.
    int num_cols;
    // number of non-empty cells in the grid.
    int num_nz;
    // stores column index for each non-empty element. indexed in [0 .. num_nz).
    int* column;
    // columns of the row R: row_offset[R] .. row_offset[R+1]. indexed in [0 .. num_rows + 1).
    int* row_offset;
    // 4 neighbour indices per non-empty element (index + 1, or 0 if no non-empty neis). indexed in [0 .. num_nz)
    int* neis;
};

}

#endif
