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

#include <stdio.h> // temp

#include <algorithm>
#include <math.h>
#include <float.h>
#include <string.h>
#include "corridormap/assert.h"
#include "corridormap/memory.h"
#include "corridormap/render_interface.h"
#include "corridormap/runtime_types.h"
#include "corridormap/build.h"

namespace corridormap {

namespace
{
    const float CORRIDORMAP_SQRT_2 = 1.41421356f;
    const float CORRIDORMAP_PI     = 3.14159265f;

    inline vec2 make_vec2(float x, float y)
    {
        vec2 result = { x, y };
        return result;
    }

    inline vec2 add(vec2 a, vec2 b)
    {
        return make_vec2(a.x + b.x, a.y + b.y);
    }

    inline vec2 sub(vec2 a, vec2 b)
    {
        return make_vec2(a.x - b.x, a.y - b.y);
    }

    inline vec2 scale(vec2 a, float val)
    {
        return make_vec2(a.x*val, a.y*val);
    }

    inline vec2 normalized(vec2 a)
    {
        return scale(a, 1.f/sqrt(a.x*a.x + a.y*a.y));
    }

    inline float dot(vec2 a, vec2 b)
    {
        return a.x*b.x + a.y*b.y;
    }

    inline float len(vec2 a)
    {
        return sqrt(dot(a, a));
    }

    inline float clamp(float val, float a, float b)
    {
        return std::min(std::max(val, a), b);
    }
}

bbox2 bounds(const footprint& f, float border)
{
    const float* x = f.x;
    const float* y = f.y;
    const int num_verts = f.num_verts;

    bbox2 result;
    result.min[0] = +FLT_MAX;
    result.min[1] = +FLT_MAX;
    result.max[0] = -FLT_MAX;
    result.max[1] = -FLT_MAX;

    for (int i = 0; i < num_verts; ++i)
    {
        result.min[0] = std::min(x[i], result.min[0]);
        result.min[1] = std::min(y[i], result.min[1]);
        result.max[0] = std::max(x[i], result.max[0]);
        result.max[1] = std::max(y[i], result.max[1]);
    }

    result.min[0] -= border;
    result.min[1] -= border;
    result.max[0] += border;
    result.max[1] += border;

    return result;
}

float max_distance(bbox2 scene_bbox)
{
    float w = scene_bbox.max[0] - scene_bbox.min[0];
    float h = scene_bbox.max[1] - scene_bbox.min[1];
    return std::max(w, h) * CORRIDORMAP_SQRT_2;
}

int vertex_distance_mesh_tris(float max_dist, float max_error)
{
    float cone_half_angle = acos((max_dist-max_error)/max_dist);
    unsigned cone_triangle_count = static_cast<unsigned>(ceil(CORRIDORMAP_PI/cone_half_angle));
    return cone_triangle_count;
}

int max_distance_mesh_verts(const footprint& f, float max_dist, float max_error)
{
    int point_tris = vertex_distance_mesh_tris(max_dist, max_error);
    // point_tris triangles per vertex plus 4 triangles per edge plus four border planes.
    return point_tris*f.num_verts*3 + f.num_verts*4*3 + 6*4;
}

namespace
{
    inline int build_cone_sector(render_vertex*& output, vec2 pos, int steps, float step_angle, float start_angle, float radius)
    {
        int nverts = 0;

        for (int i = 0; i < steps; ++i, nverts += 3)
        {
            render_vertex* a = output++;
            render_vertex* b = output++;
            render_vertex* c = output++;

            a->x = pos.x;
            a->y = pos.y;
            a->z = 0.f;

            b->x = pos.x + radius*cos(start_angle + (i + 0)*step_angle);
            b->y = pos.y + radius*sin(start_angle + (i + 0)*step_angle);
            b->z = radius;

            c->x = pos.x + radius*cos(start_angle + (i + 1)*step_angle);
            c->y = pos.y + radius*sin(start_angle + (i + 1)*step_angle);
            c->z = radius;
        }

        return nverts;
    }

    inline int build_tent_side(render_vertex*& output, vec2 a, vec2 b, float len, float size)
    {
        vec2 e = sub(b, a);
        vec2 n = scale(make_vec2(-e.y, e.x), 1.f/len);

        render_vertex p0 = { a.x, a.y, 0.f };
        render_vertex p1 = { b.x, b.y, 0.f };
        render_vertex p2 = { a.x + size*n.x, a.y + size*n.y, size };
        render_vertex p3 = { b.x + size*n.x, b.y + size*n.y, size };

        *output++ = p0; *output++ = p1; *output++ = p2;
        *output++ = p2; *output++ = p1; *output++ = p3;

        return 6;
    }
}

void build_distance_mesh(const footprint& in, bbox2 bbox, float max_dist, float max_error, distance_mesh& out)
{
    corridormap_assert(max_dist > max_error);

    const float cone_half_angle = acos((max_dist-max_error) / max_dist);
    const int cone_triangle_count = static_cast<unsigned>(ceil(CORRIDORMAP_PI/cone_half_angle));
    const float cone_angle = 2.f*CORRIDORMAP_PI/cone_triangle_count;

    // input
    const float* poly_x = in.x;
    const float* poly_y = in.y;
    const int* num_poly_verts = in.num_poly_verts;
    const int num_polys = in.num_polys;

    // output
    render_vertex* verts = out.verts;
    unsigned int* segment_colors = out.segment_colors;
    int* num_segment_verts = out.num_segment_verts;

    for (int i = 0; i < num_polys; ++i)
    {
        int npverts = num_poly_verts[i];
        int nsegverts = 0;

        int prev_idx = npverts - 2;
        int curr_idx = npverts - 1;
        int next_idx = 0;

        for (; next_idx < npverts; prev_idx = curr_idx, curr_idx = next_idx++)
        {
            vec2 prev = { poly_x[prev_idx], poly_y[prev_idx] };
            vec2 curr = { poly_x[curr_idx], poly_y[curr_idx] };
            vec2 next = { poly_x[next_idx], poly_y[next_idx] };

            float len_e1 = len(sub(next, curr));

            vec2 e0 = normalized(sub(prev, curr));
            vec2 e1 = normalized(sub(next, curr));

            float cos_inner = dot(e0, e1);
            float angle_inner = acos(cos_inner);
            float angle_cone_sector = 2.f * CORRIDORMAP_PI - angle_inner;

            int angle_cone_sector_steps = static_cast<unsigned>(ceil(angle_cone_sector/cone_angle));
            float angle_cone_sector_step = angle_cone_sector/angle_cone_sector_steps;
            float angle_start = atan2(e0.y, e0.x);

            // 1. generate cone sector for the current vertex.
            nsegverts += build_cone_sector(verts, curr, angle_cone_sector_steps, angle_cone_sector_step, angle_start, max_dist);

            // 2. generate tent for (curr, next) edge.
            nsegverts += build_tent_side(verts, curr, next, len_e1, max_dist);
            nsegverts += build_tent_side(verts, next, curr, len_e1, max_dist);
        }

        poly_x += npverts;
        poly_y += npverts;

        *(segment_colors++)    = i;
        *(num_segment_verts++) = nsegverts;
    }

    // 3. generate borders.
    {
        vec2 lt = { bbox.min[0], bbox.max[1] };
        vec2 lb = { bbox.min[0], bbox.min[1] };
        vec2 rt = { bbox.max[0], bbox.max[1] };
        vec2 rb = { bbox.max[0], bbox.min[1] };

        vec2 len = sub(rt, lb);

        *(num_segment_verts++) = build_tent_side(verts, lb, rb, len.x, max_dist);
        *(num_segment_verts++) = build_tent_side(verts, rb, rt, len.y, max_dist);
        *(num_segment_verts++) = build_tent_side(verts, rt, lt, len.x, max_dist);
        *(num_segment_verts++) = build_tent_side(verts, lt, lb, len.y, max_dist);

        for (int i = 0; i < num_border_segments; ++i)
        {
            *(segment_colors++) = num_polys + i;
        }
    }

    out.num_segments = num_border_segments + in.num_polys;
    out.num_verts = static_cast<int>(verts - out.verts);
}

void render_distance_mesh(renderer* render_iface, const distance_mesh& mesh)
{
    render_iface->begin();

    int vertices_offset = 0;

    for (int i = 0; i < mesh.num_segments; ++i)
    {
        int num_verts = mesh.num_segment_verts[i];
        unsigned color = mesh.segment_colors[i];
        const render_vertex* vertices = mesh.verts + vertices_offset;
        render_iface->draw(vertices, num_verts/3, color);
        vertices_offset += num_verts;
    }

    render_iface->end();
}

void set_segment_colors(distance_mesh& mesh, unsigned int* colors, int ncolors)
{
    for (int i = 0; i < mesh.num_segments; ++i)
    {
        mesh.segment_colors[i] = colors[i % ncolors];
    }
}

namespace
{
    inline void store_edge_normal(vec2 edge_from, vec2 edge_to, float* out_x, float* out_y)
    {
        vec2 dir = normalized(sub(edge_from, edge_to));
        *(out_x++) = +dir.y;
        *(out_y++) = -dir.x;
    }
}

void build_footprint_normals(const footprint& in, bbox2 bbox, footprint_normals& out)
{
    const float* poly_x = in.x;
    const float* poly_y = in.y;
    const int* num_poly_verts = in.num_poly_verts;
    const int num_polys = in.num_polys;

    float* normal_x = out.x;
    float* normal_y = out.y;
    int* num_obstacle_normals = out.num_obstacle_normals;
    int* obstacle_normal_offsets = out.obstacle_normal_offsets;

    int num_normals = 0;

    for (int i = 0; i < num_polys; ++i)
    {
        *(obstacle_normal_offsets++) = num_normals;

        int nverts = num_poly_verts[i];

        int curr_idx = nverts - 1;
        int next_idx = 0;

        for (; next_idx < nverts; curr_idx = next_idx++, ++num_normals)
        {
            vec2 curr = { poly_x[curr_idx], poly_y[curr_idx] };
            vec2 next = { poly_x[next_idx], poly_y[next_idx] };
            store_edge_normal(curr, next, normal_x, normal_y);
        }

        *(num_obstacle_normals++) = nverts;
    }

    {
        vec2 lt = { bbox.min[0], bbox.max[1] };
        vec2 lb = { bbox.min[0], bbox.min[1] };
        vec2 rt = { bbox.max[0], bbox.max[1] };
        vec2 rb = { bbox.max[0], bbox.min[1] };

        store_edge_normal(lb, rb, normal_x, normal_y);
        store_edge_normal(rb, rt, normal_x, normal_y);
        store_edge_normal(rt, lt, normal_x, normal_y);
        store_edge_normal(lt, lb, normal_x, normal_y);

        for (int i = 0; i < num_border_segments; ++i)
        {
            *(obstacle_normal_offsets++) = num_normals + i;
            *(num_obstacle_normals++) = 1;
        }
    }
}

namespace
{
    int find_normal_index(
        const float* vertex_x,
        const float* vertex_y,
        const float* normal_x,
        const float* normal_y,
        const int* num_obstacle_normals,
        const int* obstacle_normal_offsets,
        const int obstacle_id,
        const vec2 edge_point)
    {
        int num_normals = num_obstacle_normals[obstacle_id];
        int first_normal_idx = obstacle_normal_offsets[obstacle_id];
        int last_normal_idx = first_normal_idx + num_normals - 1;

        int curr_idx = last_normal_idx;
        int next_idx = first_normal_idx;

        for (; next_idx <= last_normal_idx; curr_idx = next_idx++)
        {
            vec2 vertex = { vertex_x[curr_idx], vertex_y[curr_idx] };
            vec2 normal_curr = { normal_x[curr_idx], normal_y[curr_idx] };
            vec2 normal_next = { normal_x[next_idx], normal_y[next_idx] };

            vec2 mid = normalized(scale(add(normal_curr, normal_next), 0.5f));
            vec2 dir = normalized(sub(edge_point, vertex));

            float dot_n = dot(normal_curr, mid);
            float dot_d = dot(dir, mid);

            if (dot_d >= dot_n)
            {
                return curr_idx + 1;
            }
        }

        return 0;
    }
}

void build_edge_point_normal_indices(const voronoi_features& features, const footprint& obstacles, const footprint_normals& normals, voronoi_edge_normals& out)
{
    const int grid_width = features.grid_width;
    const int num_edge_points = features.num_edge_points;
    const unsigned int* edges = features.edges;
    const unsigned int* obstacle_ids_left = features.edge_obstacle_ids_left;
    const unsigned int* obstacle_ids_right = features.edge_obstacle_ids_right;

    const float* vertex_x = obstacles.x;
    const float* vertex_y = obstacles.y;

    const float* normal_x = normals.x;
    const float* normal_y = normals.y;
    const int* num_obstacle_normals = normals.num_obstacle_normals;
    const int* obstacle_normal_offsets = normals.obstacle_normal_offsets;

    int* normal_indices_left = out.edge_normal_indices_left;
    int* normal_indices_right = out.edge_normal_indices_right;

    for (int i = 0; i < num_edge_points; ++i)
    {
        unsigned int edge_point_idx = edges[i];
        unsigned int obstacle_left = obstacle_ids_left[i];
        unsigned int obstacle_right = obstacle_ids_right[i];

        vec2 edge_point = { float(edge_point_idx%grid_width), float(edge_point_idx/grid_width) };

        int left_idx  = find_normal_index(vertex_x, vertex_y, normal_x, normal_y, num_obstacle_normals, obstacle_normal_offsets, obstacle_left, edge_point);
        int right_idx = find_normal_index(vertex_x, vertex_y, normal_x, normal_y, num_obstacle_normals, obstacle_normal_offsets, obstacle_right, edge_point);

        normal_indices_left[i]  = left_idx;
        normal_indices_right[i] = right_idx;
    }
}

void compute_closest_points(const footprint& obstacles, const int* obstacle_offsets, const float* pos_x, const float* pos_y,
                            const unsigned int* obstacle_ids, const int num_points, float* out_x, float* out_y)
{
    const float* obst_x = obstacles.x;
    const float* obst_y = obstacles.y;
    const int* num_obst_verts = obstacles.num_poly_verts;

    for (int i = 0; i < num_points; ++i)
    {
        vec2 point = { pos_x[i], pos_y[i] };
        vec2 closest = { FLT_MAX, FLT_MAX };

        float min_dist = FLT_MAX;

        unsigned int obstacle_id = obstacle_ids[i];
        int first_vertex_idx = obstacle_offsets[obstacle_id];
        int last_vertex_idx = first_vertex_idx + num_obst_verts[obstacle_id] - 1;

        int curr_idx = last_vertex_idx;
        int next_idx = first_vertex_idx;

        for (; next_idx <= last_vertex_idx; curr_idx = next_idx++)
        {
            vec2 p0 = { obst_x[curr_idx], obst_y[curr_idx] };
            vec2 p1 = { obst_x[next_idx], obst_y[next_idx] };

            vec2 seg = sub(p1, p0);
            vec2 dir = normalized(seg);
            float seg_len = len(seg);

            float proj = dot(sub(point, p0), dir);

            vec2 seg_closest = add(p0, scale(dir, clamp(proj, 0.f, seg_len)));

            vec2 seg_closest_dir = sub(seg_closest, point);
            float seg_closest_dist = dot(seg_closest_dir, seg_closest_dir);

            if (seg_closest_dist < min_dist)
            {
                min_dist = seg_closest_dist;
                closest = seg_closest;
            }
        }

        out_x[i] = closest.x;
        out_y[i] = closest.y;
    }
}

void build_csr(const unsigned int* nz_coords, csr_grid& out)
{
    int* column = out.column;
    int* row_offset = out.row_offset;

    const int num_cols = out.num_cols;
    const int num_rows = out.num_rows;
    const int num_nz = out.num_nz;

    int next_row = 0;

    for (int i = 0; i < num_nz; ++i)
    {
        unsigned int coord = nz_coords[i];

        column[i] = coord % num_cols;
        int curr_row = coord / num_cols;

        for (int j = next_row; j <= curr_row; ++j)
        {
            row_offset[j] = i;
        }

        next_row = curr_row + 1;
    }

    for (int j = next_row; j < num_rows + 1; ++j)
    {
        row_offset[j] = num_nz;
    }
}

int nz(const csr_grid& grid, int row, int col)
{
    const int* column = grid.column;
    int row_b = grid.row_offset[row + 0];
    int row_e = grid.row_offset[row + 1];

    for (int i = row_b; i < row_e; ++i)
    {
        if (column[i] == col)
        {
            return i;
        }
    }

    return grid.num_nz;
}

int nz(const csr_grid& grid, int linear_index)
{
    return nz(grid, linear_index / grid.num_cols, linear_index % grid.num_cols);
}

namespace
{
    int nei_offset_row[] = { +0, -1, +1, +0 };
    int nei_offset_col[] = { -1, +0, +0, +1 };
}

csr_grid_neis cell_neis(const csr_grid& grid, int row, int col)
{
    int num_rows = grid.num_rows;
    int num_cols = grid.num_cols;

    csr_grid_neis neis;
    neis.num = 0;

    for (int i = 0; i < sizeof(nei_offset_row)/sizeof(nei_offset_row[0]); ++i)
    {
        int n_r = row + nei_offset_row[i];
        int n_c = col + nei_offset_col[i];

        if (n_r < 0 || n_r >= num_rows)
        {
            continue;
        }

        if (n_c < 0 || n_c >= num_cols)
        {
            continue;
        }

        int nz_idx = nz(grid, n_r, n_c);

        if (nz_idx < grid.num_nz)
        {
            neis.row[neis.num] = n_r;
            neis.col[neis.num] = n_c;
            neis.nz_idx[neis.num] = nz_idx;
            neis.num++;
        }
    }

    return neis;
}

csr_grid_neis cell_neis(const csr_grid& grid, int linear_index)
{
    return cell_neis(grid, linear_index / grid.num_cols, linear_index % grid.num_cols);
}

namespace
{
    template <typename T>
    struct queue
    {
        queue(memory* mem, int max_size)
            : front(0)
            , size(0)
            , max_size(max_size)
            , mem(mem)
        {
            data = allocate<T>(mem, max_size);
        }

        int front;
        int size;
        int max_size;
        memory* mem;
        T* data;
    };

    template <typename T>
    int size(const queue<T>& q)
    {
        return q.size;
    }

    template <typename T>
    void enqueue(queue<T>& q, const T val)
    {
        int idx = (q.front + q.size) % q.max_size;
        q.data[idx] = val;
        q.size += 1;
    }

    template <typename T>
    T dequeue(queue<T>& q)
    {
        T val = q.data[q.front % q.max_size];
        q.front++;
        q.size--;
        return val;
    }

    template <typename T>
    void clear(queue<T>& q)
    {
        q.front = 0;
        q.size = 0;
    }

    void trace_edges(const csr_grid& vertices, const csr_grid& edges, queue<int>& queue_edge,
                     queue<int>& queue_vert, alloc_scope<char>& visited_edge, alloc_scope<char>& visited_vert, int start_vert,
                     float* u_x, float* u_y, float* v_x, float* v_y, int& count)
    {
        int start_vert_row = start_vert / edges.num_cols;
        int start_vert_col = start_vert % edges.num_cols;

        csr_grid_neis neis = cell_neis(edges, start_vert);

        clear(queue_edge);

        for (int i = 0; i < neis.num; ++i)
        {
            enqueue(queue_edge, neis.row[i]*edges.num_cols + neis.col[i]);
            visited_edge[neis.nz_idx[i]] = 1;
        }

        int seen_verts[max_grid_neis];
        int seen_vert_count = 0;

        while (size(queue_edge) > 0 && seen_vert_count < max_grid_neis)
        {
            int edge_pt = dequeue(queue_edge);

            visited_edge[nz(edges, edge_pt)] = 1;

            csr_grid_neis vert_neis = cell_neis(vertices, edge_pt);

            bool pushed_verts = false;

            for (int i = 0; i < vert_neis.num; ++i)
            {
                int row = vert_neis.row[i];
                int col = vert_neis.col[i];
                int idx = row*vertices.num_cols + col;
                int vert_nz_idx = vert_neis.nz_idx[i];

                if (visited_vert[vert_nz_idx] != 1)
                {
                    pushed_verts = true;
                    enqueue(queue_vert, idx);
                }

                bool seen_vert = false;

                for (int k = 0; k < seen_vert_count; ++k)
                {
                    if (seen_verts[k] == idx)
                    {
                        seen_vert = true;
                        break;
                    }
                }

                if (start_vert != idx && !seen_vert)
                {
                    printf("[%d, %d] -> [%d, %d]\n", start_vert_col, start_vert_row, col, row);
                    u_x[count] = static_cast<float>(start_vert_col);
                    u_y[count] = static_cast<float>(start_vert_row);
                    v_x[count] = static_cast<float>(col);
                    v_y[count] = static_cast<float>(row);
                    count++;

                    seen_verts[seen_vert_count++] = idx;
                }
            }

            if (pushed_verts)
            {
                continue;
            }

            csr_grid_neis neis = cell_neis(edges, edge_pt);

            for (int i = 0; i < neis.num; ++i)
            {
                int nz_idx = neis.nz_idx[i];
                int row = neis.row[i];
                int col = neis.col[i];
                int idx = row*vertices.num_cols + col;

                if (visited_edge[nz_idx] != 1)
                {
                    enqueue(queue_edge, idx);
                }
            }
        }
    }
}

void trace_edges(memory* scratch, const csr_grid& vertices, const csr_grid& edges, int start_vert,
                 float* u_x, float* u_y, float* v_x, float* v_y, int& count)
{
    alloc_scope<char> visited_edge(scratch, edges.num_nz);
    alloc_scope<char> visited_vert(scratch, vertices.num_nz);
    zero_mem(visited_edge);
    zero_mem(visited_vert);

    queue<int> queue_edge(scratch, edges.num_nz);
    queue<int> queue_vert(scratch, vertices.num_nz);

    enqueue(queue_vert, start_vert);
    visited_vert[nz(vertices, start_vert)] = 1;

    while (size(queue_vert) > 0)
    {
        int vert = dequeue(queue_vert);
        visited_vert[nz(vertices, vert)] = 1;

        trace_edges(vertices, edges, queue_edge, queue_vert, visited_edge, visited_vert, vert, u_x, u_y, v_x, v_y, count);
    }
}

}
