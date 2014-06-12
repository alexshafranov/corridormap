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

#include <algorithm>
#include <math.h>
#include <float.h>
#include <string.h>
#include "corridormap/assert.h"
#include "corridormap/memory.h"
#include "corridormap/render_interface.h"
#include "corridormap/runtime_types.h"
#include "corridormap/build.h"
#include "corridormap/vec2.h"

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

float max_distance(bbox2 bounds)
{
    float w = bounds.max[0] - bounds.min[0];
    float h = bounds.max[1] - bounds.min[1];
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

void build_distance_mesh(const footprint& in, bbox2 bounds, float max_dist, float max_error, distance_mesh& out)
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

        *segment_colors++    = i;
        *num_segment_verts++ = nsegverts;
    }

    // 3. generate borders.
    {
        vec2 lt = { bounds.min[0], bounds.max[1] };
        vec2 lb = { bounds.min[0], bounds.min[1] };
        vec2 rt = { bounds.max[0], bounds.max[1] };
        vec2 rb = { bounds.max[0], bounds.min[1] };

        vec2 len = sub(rt, lb);

        *num_segment_verts++ = build_tent_side(verts, lb, rb, len.x, max_dist);
        *num_segment_verts++ = build_tent_side(verts, rb, rt, len.y, max_dist);
        *num_segment_verts++ = build_tent_side(verts, rt, lt, len.x, max_dist);
        *num_segment_verts++ = build_tent_side(verts, lt, lb, len.y, max_dist);

        for (int i = 0; i < num_border_segments; ++i)
        {
            *segment_colors++ = num_polys + i;
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
        *out_x++ = +dir.y;
        *out_y++ = -dir.x;
    }
}

void build_footprint_normals(const footprint& in, bbox2 bounds, footprint_normals& out)
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
        *obstacle_normal_offsets++ = num_normals;

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

        *num_obstacle_normals++ = nverts;
    }

    {
        vec2 lt = { bounds.min[0], bounds.max[1] };
        vec2 lb = { bounds.min[0], bounds.min[1] };
        vec2 rt = { bounds.max[0], bounds.max[1] };
        vec2 rb = { bounds.max[0], bounds.min[1] };

        store_edge_normal(lb, rb, normal_x, normal_y);
        store_edge_normal(rb, rt, normal_x, normal_y);
        store_edge_normal(rt, lt, normal_x, normal_y);
        store_edge_normal(lt, lb, normal_x, normal_y);

        for (int i = 0; i < num_border_segments; ++i)
        {
            *obstacle_normal_offsets++ = num_normals + i;
            *num_obstacle_normals++ = 1;
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

    int get_next_point(const csr_grid& edges, const voronoi_features& features, int current_point, int previous_point)
    {
        unsigned int side_1 = features.edge_obstacle_ids_1[nz(edges, current_point)];
        unsigned int side_2 = features.edge_obstacle_ids_2[nz(edges, current_point)];

        csr_grid_neis neis = cell_neis(edges, current_point);

        for (int i = 0; i < neis.num; ++i)
        {
            if (neis.lin_idx[i] == previous_point)
            {
                continue;
            }

            unsigned int nei_side_1 = features.edge_obstacle_ids_1[neis.nz_idx[i]];
            unsigned int nei_side_2 = features.edge_obstacle_ids_2[neis.nz_idx[i]];

            if (nei_side_1 == side_1 && nei_side_2 == side_2 ||
                nei_side_1 == side_2 && nei_side_2 == side_1)
            {
                return neis.lin_idx[i];
            }
        }

        return -1;
    }

    int get_neigbour_vertex(const csr_grid& vertices, int cell_linear_index, int start_vert)
    {
        csr_grid_neis neis = cell_neis(vertices, cell_linear_index);

        for (int i = 0; i < neis.num; ++i)
        {
            if (neis.lin_idx[i] != start_vert)
            {
                return neis.lin_idx[i];
            }
        }

        return -1;
    }

    void fix_sides(voronoi_features& features, voronoi_edge_normals& normals, int prev_point_nz, int curr_point_nz)
    {
        unsigned int ps1 = features.edge_obstacle_ids_1[prev_point_nz];

        unsigned int cs1 = features.edge_obstacle_ids_1[curr_point_nz];
        unsigned int cs2 = features.edge_obstacle_ids_2[curr_point_nz];

        int ncs1 = normals.edge_normal_indices_1[curr_point_nz];
        int ncs2 = normals.edge_normal_indices_2[curr_point_nz];

        if (ps1 != cs1)
        {
            std::swap(cs1, cs2);
            std::swap(ncs1, ncs2);
        }

        features.edge_obstacle_ids_1[curr_point_nz] = cs1;
        features.edge_obstacle_ids_2[curr_point_nz] = cs2;

        normals.edge_normal_indices_1[curr_point_nz] = ncs1;
        normals.edge_normal_indices_2[curr_point_nz] = ncs2;
    }

    int trace_incident_edge(const csr_grid& vertices, const csr_grid& edges, voronoi_features& features, voronoi_edge_normals& normals,
                            char* visited_edges, int start_vert, int edge_point)
    {
        int prev = edge_point;

        for (int curr = edge_point; curr >= 0;)
        {
            if (visited_edges[nz(edges, curr)])
            {
                return -1;
            }

            visited_edges[nz(edges, curr)] = 1;

            int vert = get_neigbour_vertex(vertices, curr, start_vert);

            if (vert >= 0)
            {
                return vert;
            }

            fix_sides(features, normals, nz(edges, prev), nz(edges, curr));

            int next = get_next_point(edges, features, curr, prev);
            prev = curr;
            curr = next;
        }

        return -1;
    }

    int trace_event_points(const csr_grid& vertices, const csr_grid& edges, const voronoi_features& features, const voronoi_edge_normals& normals,
                           int start_vert, int edge_point, int* events)
    {
        int num_events = 0;
        int prev = edge_point;

        for (int curr = edge_point; curr >= 0;)
        {
            if (get_neigbour_vertex(vertices, curr, start_vert) >= 0)
            {
                break;
            }

            int p_n1 = normals.edge_normal_indices_1[nz(edges, prev)];
            int p_n2 = normals.edge_normal_indices_2[nz(edges, prev)];
            int c_n1 = normals.edge_normal_indices_1[nz(edges, curr)];
            int c_n2 = normals.edge_normal_indices_2[nz(edges, curr)];

            if (p_n1 != c_n1 || p_n2 != c_n2)
            {
                events[num_events] = curr;
                num_events++;
            }

            int next = get_next_point(edges, features, curr, prev);
            prev = curr;
            curr = next;
        }

        return num_events;
    }
}

void trace_edges(memory* scratch, const csr_grid& vertices, const csr_grid& edges,
                 voronoi_edge_normals& edge_normal_indices, voronoi_features& features, voronoi_traced_edges& out)
{
    queue<int> queue_vert(scratch, vertices.num_nz);
    alloc_scope<char> visited_vert(scratch, vertices.num_nz);
    alloc_scope<char> visited_edge(scratch, edges.num_nz);
    zero_mem(visited_vert);
    zero_mem(visited_edge);

    int start_vert = features.verts[0];

    enqueue(queue_vert, start_vert);
    visited_vert[nz(vertices, start_vert)] = 1;

    int* out_u = out.u;
    int* out_v = out.v;
    int* out_events = out.events;
    int* out_event_offsets = out.edge_event_offset;
    int* out_num_events = out.edge_num_events;

    int num_edges = 0;
    int num_events = 0;

    while (size(queue_vert) > 0)
    {
        int u = dequeue(queue_vert);
        visited_vert[nz(vertices, u)] = 1;

        csr_grid_neis neis = cell_neis(edges, u);

        for (int i = 0; i < neis.num; ++i)
        {
            int v = trace_incident_edge(vertices, edges, features, edge_normal_indices, visited_edge, u, neis.lin_idx[i]);

            if (v < 0)
            {
                continue;
            }

            int num_edge_events = trace_event_points(vertices, edges, features, edge_normal_indices, u, neis.lin_idx[i], out_events + num_events);

            out_u[num_edges] = u;
            out_v[num_edges] = v;
            out_event_offsets[num_edges] = num_events;
            out_num_events[num_edges] = num_edge_events;

            num_edges++;
            num_events += num_edge_events;

            if (visited_vert[nz(vertices, v)] != 1)
            {
                enqueue(queue_vert, v);
            }
        }
    }

    out.num_edges = num_edges;
    out.num_events = num_events;
}

namespace
{
    vec2 compute_closest_point(const footprint& obstacles, const int* obstacle_offsets, unsigned int obstacle_id, const vec2& point)
    {
        const float* obst_x = obstacles.x;
        const float* obst_y = obstacles.y;
        const int* num_obst_verts = obstacles.num_poly_verts;

        vec2 closest = { FLT_MAX, FLT_MAX };

        float min_dist = FLT_MAX;

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

        return closest;
    }

    vec2 convert_from_image(int lin_idx, int grid_width, int grid_height, bbox2 bounds)
    {
        float bounds_width = bounds.max[0] - bounds.min[0];
        float bounds_height = bounds.max[1] - bounds.min[1];
        float vx = float(lin_idx % grid_width);
        float vy = float(lin_idx / grid_width);
        vx = vx / grid_width * bounds_width + bounds.min[0];
        vy = vy / grid_height * bounds_height + bounds.min[1];
        return make_vec2(vx, vy);
    }

    bool is_ccw(vec2 u, vec2 v1, vec2 v2)
    {
        vec2 d1 = sub(v1, u);
        vec2 d2 = sub(v2, u);
        return d1.x*d2.y - d2.x*d1.y > 0.f;
    }

    void add_half_edge(vertex* vertices, half_edge* half_edges, int vert, int h_edge)
    {
        int head = vertices[vert].half_edge;

        if (head == null_idx)
        {
            vertices[vert].half_edge = h_edge;
            half_edges[h_edge].next = h_edge;
        }
        else
        {
            vec2 u = vertices[vert].pos;
            vec2 v2 = vertices[half_edges[h_edge].target].pos;

            int insert_after = null_idx;
            int curr = head;

            for (;;)
            {
                vec2 v1 = vertices[half_edges[curr].target].pos;

                if (!is_ccw(u, v1, v2))
                {
                    break;
                }

                insert_after = curr;
                curr = half_edges[curr].next;

                if (curr == head)
                {
                    break;
                }
            }

            if (insert_after == null_idx)
            {
                int tail = head;

                while (half_edges[tail].next != null_idx)
                {
                    tail = half_edges[tail].next;
                }

                vertices[vert].half_edge = h_edge;
                half_edges[h_edge].next = head;
                half_edges[tail].next = h_edge;
            }
            else
            {
                int next = half_edges[insert_after].next;
                half_edges[insert_after].next = h_edge;
                half_edges[h_edge].next = next;
            }
        }
    }

    void add_event(half_edge* half_edges, event* events, int h_edge, int evt)
    {
        int dir = h_edge & 1;
        int head = half_edges[h_edge].event;

        if (head == null_idx)
        {
            half_edges[h_edge].event = evt;
            events[evt].next[dir] = null_idx;
        }
        else
        {
            int tail = head;

            while (events[tail].next[dir] != null_idx)
            {
                tail = events[tail].next[dir];
            }

            events[tail].next[dir] = evt;
            events[evt].next[dir] = null_idx;
        }
    }
}

void build_voronoi_diagram(const footprint& obstacles, const int* obstacle_offsets, bbox2 bounds, const voronoi_features& features,
                           const csr_grid& edge_grid,  const csr_grid& vertex_grid, const voronoi_traced_edges& traced_edges, voronoi_diagram& out)
{
    // 1. convert vertices from image to footprint coordinates and initialize offsets.
    for (int i = 0; i < features.num_vert_points; ++i)
    {
        int vert = features.verts[i];
        out.vertices.elems[i].pos = convert_from_image(vert, features.grid_width, features.grid_height, bounds);
        out.vertices.elems[i].next = null_idx;
        out.vertices.elems[i].half_edge = null_idx;
    }

    // 2. compute vertex sides.
    for (int i = 0; i < features.num_vert_points; ++i)
    {
        vec2 point = out.vertices.elems[i].pos;

        for (int j = 0; j < 4; ++j)
        {
            unsigned int obstacle_id = features.vert_obstacle_ids[i*4 + j];
            vec2 closest = compute_closest_point(obstacles, obstacle_offsets, obstacle_id, point);
            out.vertices.elems[i].sides[j] = closest;
        }
    }

    // 3. convert events from image to footprint coordinates.
    for (int i = 0; i < traced_edges.num_events; ++i)
    {
        int event = traced_edges.events[i];
        out.events.elems[i].pos = convert_from_image(event, features.grid_width, features.grid_height, bounds);
        out.events.elems[i].next[0] = null_idx;
        out.events.elems[i].next[1] = null_idx;
    }

    // 4. compute event sides.
    for (int i = 0; i < traced_edges.num_events; ++i)
    {
        vec2 point = out.events.elems[i].pos;
        int event = traced_edges.events[i];
        unsigned int obstacle_id_0 = features.edge_obstacle_ids_1[nz(edge_grid, event)];
        unsigned int obstacle_id_1 = features.edge_obstacle_ids_2[nz(edge_grid, event)];
        out.events.elems[i].sides[0] = compute_closest_point(obstacles, obstacle_offsets, obstacle_id_0, point);
        out.events.elems[i].sides[1] = compute_closest_point(obstacles, obstacle_offsets, obstacle_id_1, point);
    }

    // 5. topology.
    for (int i = 0; i < traced_edges.num_edges; ++i)
    {
        int u = nz(vertex_grid, traced_edges.u[i]);
        int v = nz(vertex_grid, traced_edges.v[i]);

        out.half_edges.elems[i*2 + 0].target = v;
        out.half_edges.elems[i*2 + 1].target = u;
        out.half_edges.elems[i*2 + 0].event = null_idx;
        out.half_edges.elems[i*2 + 1].event = null_idx;

        add_half_edge(out.vertices.elems, out.half_edges.elems, u, i*2 + 0);
        add_half_edge(out.vertices.elems, out.half_edges.elems, v, i*2 + 1);
    }

    // 6. events.
    for (int i = 0; i < traced_edges.num_edges; ++i)
    {
        int event_offset = traced_edges.edge_event_offset[i];
        int num_events = traced_edges.edge_num_events[i];

        for (int j = 0; j < num_events; ++j)
        {
            add_event(out.half_edges.elems, out.events.elems, i*2 + 0, event_offset + j);
        }

        for (int j = num_events - 1; j >= 0; --j)
        {
            add_event(out.half_edges.elems, out.events.elems, i*2 + 1, event_offset + j);
        }
    }
}

}
