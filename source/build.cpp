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

#include <stdio.h>

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

int distance_mesh_tris_for_point(float max_dist, float max_error)
{
    float cone_half_angle = acos((max_dist-max_error)/max_dist);
    unsigned cone_triangle_count = static_cast<unsigned>(ceil(CORRIDORMAP_PI/cone_half_angle));
    return cone_triangle_count;
}

int max_distance_mesh_verts(const footprint& f, float max_dist, float max_error)
{
    int point_tris = distance_mesh_tris_for_point(max_dist, max_error);
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
    inline void store_edge_normal(vec2 u, vec2 v, float*& out_x, float*& out_y)
    {
        vec2 dir = normalized(sub(v, u));
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

        poly_x += nverts;
        poly_y += nverts;

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
        int oid = obstacle_id;

        int num_normals = num_obstacle_normals[oid];

        int first_normal_idx = obstacle_normal_offsets[oid];
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

void build_edge_point_normal_indices(const voronoi_features& features, const footprint& obstacles, const footprint_normals& normals, bbox2 bounds, voronoi_edge_normals& out)
{
    const int grid_width = features.grid_width;
    const int grid_height = features.grid_height;
    const int num_edge_points = features.num_edge_points;
    const unsigned int* edges = features.edges;
    const unsigned int* obstacle_ids_1 = features.edge_obstacle_ids_1;
    const unsigned int* obstacle_ids_2 = features.edge_obstacle_ids_2;

    const float* vertex_x = obstacles.x;
    const float* vertex_y = obstacles.y;

    const float* normal_x = normals.x;
    const float* normal_y = normals.y;
    const int* num_obstacle_normals = normals.num_obstacle_normals;
    const int* obstacle_normal_offsets = normals.obstacle_normal_offsets;

    int* normal_indices_1 = out.edge_normal_indices_1;
    int* normal_indices_2 = out.edge_normal_indices_2;

    float bounds_width = bounds.max[0] - bounds.min[0];
    float bounds_height = bounds.max[1] - bounds.min[1];

    for (int i = 0; i < num_edge_points; ++i)
    {
        unsigned int edge_point_idx = edges[i];
        unsigned int obstacle_1 = obstacle_ids_1[i];
        unsigned int obstacle_2 = obstacle_ids_2[i];

        int edge_point_x = edge_point_idx%grid_width;
        int edge_point_y = edge_point_idx/grid_width;

        vec2 edge_point = { bounds.min[0] + float(edge_point_x)/grid_width * bounds_width, bounds.min[1] + float(edge_point_y)/grid_height * bounds_height};

        normal_indices_1[i] = find_normal_index(vertex_x, vertex_y, normal_x, normal_y, num_obstacle_normals, obstacle_normal_offsets, obstacle_1, edge_point);
        normal_indices_2[i] = find_normal_index(vertex_x, vertex_y, normal_x, normal_y, num_obstacle_normals, obstacle_normal_offsets, obstacle_2, edge_point);
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
            neis.lin_idx[neis.num] = n_r*num_cols + n_c;
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

        ~queue()
        {
            mem->deallocate(data);
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

    struct tracing_state
    {
        tracing_state(memory* mem, int num_verts, int num_edges)
            : queue_edge(mem, num_edges)
            , queue_vert(mem, num_verts)
            , visited_edge(mem, num_edges)
            , visited_vert(mem, num_verts)
            , parent(mem, num_edges)
        {
            zero_mem(visited_edge);
            zero_mem(visited_vert);
            zero_mem(parent);
        }

        queue<int> queue_edge;
        queue<int> queue_vert;
        alloc_scope<char> visited_edge;
        alloc_scope<char> visited_vert;
        alloc_scope<int>  parent;
    };

    int linear_index(const csr_grid& grid, int nz_index)
    {
        int col = grid.column[nz_index];
        int row = 0;

        for (; row < grid.num_rows; ++row)
        {
            if (nz_index < grid.row_offset[row+1])
            {
                break;
            }
        }

        return row*grid.num_cols + col;
    }

    void trace_incident_edges(const csr_grid& vertices, const csr_grid& edges, const voronoi_edge_normals& normal_indices, int start_vert, tracing_state& state, voronoi_traced_edges& out, const voronoi_features& features)
    {
        int* out_u = out.u;
        int* out_v = out.v;
        int* out_events = out.events;

        csr_grid_neis neis = cell_neis(edges, start_vert);

        clear(state.queue_edge);
        zero_mem(state.parent);

        for (int i = 0; i < neis.num; ++i)
        {
            state.parent[neis.nz_idx[i]] = -1;
            state.visited_edge[neis.nz_idx[i]] = 1;
            enqueue(state.queue_edge, neis.lin_idx[i]);
        }

        int seen_verts[max_grid_neis];
        int seen_vert_count = 0;
        int num_edges = out.num_edges;

        while (size(state.queue_edge) > 0 && seen_vert_count < max_grid_neis)
        {
            int edge_pt = dequeue(state.queue_edge);

            state.visited_edge[nz(edges, edge_pt)] = 1;

            csr_grid_neis vert_neis = cell_neis(vertices, edge_pt);

            bool pushed_verts = false;

            for (int i = 0; i < vert_neis.num; ++i)
            {
                int vert = vert_neis.lin_idx[i];
                int vert_nz = vert_neis.nz_idx[i];

                if (state.visited_vert[vert_nz] != 1)
                {
                    pushed_verts = true;
                    enqueue(state.queue_vert, vert);
                }

                bool seen_vert = false;

                for (int k = 0; k < seen_vert_count; ++k)
                {
                    if (seen_verts[k] == vert)
                    {
                        seen_vert = true;
                        break;
                    }
                }

                if (start_vert != vert && !seen_vert)
                {
                    out_u[num_edges] = start_vert;
                    out_v[num_edges] = vert;
                    num_edges++;

                    seen_verts[seen_vert_count++] = vert;

                    int num_events = 0;
                    int prev = nz(edges, edge_pt);
                    int curr = state.parent[prev];

                    for (; curr != -1; prev = curr, curr = state.parent[curr])
                    {
                        unsigned int prev_color_l = features.edge_obstacle_ids_1[prev];
                        unsigned int curr_color_l = features.edge_obstacle_ids_1[curr];
                        // unsigned int prev_color_r = features.edge_obstacle_ids_right[prev];
                        unsigned int curr_color_r = features.edge_obstacle_ids_2[curr];
                        // printf("cl=%d cr=%d pl=%d pr=%d\n", curr_color_l, curr_color_r, prev_color_l, prev_color_r);

                        int prev_l = normal_indices.edge_normal_indices_1[prev];
                        int prev_r = normal_indices.edge_normal_indices_2[prev];

                        int curr_l = normal_indices.edge_normal_indices_1[curr];
                        int curr_r = normal_indices.edge_normal_indices_2[curr];

                        int swapped = 0;

                        // int lin_idx_prev = linear_index(edges, prev);
                        // printf("px=%d py=%d pcl=%d ccl=%d ccr=%d\n", lin_idx_prev%edges.num_cols, lin_idx_prev/edges.num_cols, prev_color_l, curr_color_l, curr_color_r);

                        // if (prev_color_l != curr_color_l)
                        // {
                        //     swapped = 1;
                        //     std::swap(curr_l, curr_r);
                        //     std::swap(curr_color_l, curr_color_r);
                        // }

                        // if (prev_color_l != curr_color_l)
                        // {
                        //     continue;
                        // }

                        // if (prev_color_r != curr_color_r)
                        // {
                        //     continue;
                        // }

                        // printf("pl=%d pr=%d cl=%d cr=%d\n", prev_l, prev_r, curr_l, curr_r);
                        int lin_idx = linear_index(edges, curr);
                        // int id_left = features.edge_obstacle_ids_left[curr] & 0x00111111;
                        int id_left = curr_color_l;
                        // int type = features.edge_obstacle_ids_left[curr] >> 24;
                        int id_right = curr_color_r;
                        printf("x=%d y=%d nl=%d nr=%d ol=%d or=%d s=%d\n", lin_idx%edges.num_cols, lin_idx/edges.num_cols, curr_l, curr_r, id_left, id_right, swapped);

                        if (prev_color_l != curr_color_l)
                        {
                            swapped = 1;
                            std::swap(curr_l, curr_r);
                            std::swap(curr_color_l, curr_color_r);
                        }

                        if (prev_l != curr_l)
                        {
                            num_events++;
                            out_events[out.num_events] = lin_idx;
                            out.num_events++;
                            continue;
                        }

                        if (prev_r != curr_r)
                        {
                            num_events++;
                            out_events[out.num_events] = lin_idx;
                            out.num_events++;
                            continue;
                        }
                    }

                    printf("num_events=%d\n", num_events);
                }
            }

            if (pushed_verts)
            {
                continue;
            }

            csr_grid_neis neis = cell_neis(edges, edge_pt);

            for (int i = 0; i < neis.num; ++i)
            {
                if (state.visited_edge[neis.nz_idx[i]] != 1)
                {
                    // int prev = nz(edges, edge_pt);
                    // int curr = neis.nz_idx[i];

                    // unsigned int prev_color_l = features.edge_obstacle_ids_left[prev];
                    // unsigned int prev_color_r = features.edge_obstacle_ids_right[prev];
                    // unsigned int curr_color_l = features.edge_obstacle_ids_left[curr];
                    // unsigned int curr_color_r = features.edge_obstacle_ids_right[curr];

                    // if (prev_color_l != curr_color_l)
                    // {
                    //     std::swap(curr_color_l, curr_color_r);
                    // }

                    // if (prev_color_l != curr_color_l)
                    // {
                    //     printf("ne left\n");
                    //     continue;
                    // }

                    // if (prev_color_r != curr_color_r)
                    // {
                    //     printf("ne right\n");
                    //     continue;
                    // }

                    state.parent[neis.nz_idx[i]] = nz(edges, edge_pt);
                    enqueue(state.queue_edge, neis.lin_idx[i]);
                    break;
                }
            }
        }

        out.num_edges = num_edges;
    }
}

void trace_edges(memory* scratch, const csr_grid& vertices, const csr_grid& edges,
                 const voronoi_edge_normals& edge_normal_indices, int start_vert, voronoi_traced_edges& out, const voronoi_features& features)
{
    tracing_state state(scratch, vertices.num_nz, edges.num_nz);

    enqueue(state.queue_vert, start_vert);
    state.visited_vert[nz(vertices, start_vert)] = 1;

    while (size(state.queue_vert) > 0)
    {
        int vert = dequeue(state.queue_vert);
        state.visited_vert[nz(vertices, vert)] = 1;
        trace_incident_edges(vertices, edges, edge_normal_indices, vert, state, out, features);
    }
}

}
