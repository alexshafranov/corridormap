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
#include <float.h>
#include <string.h>
#include "corridormap/assert.h"
#include "corridormap/memory.h"
#include "corridormap/render_interface.h"
#include "corridormap/vec2.h"
#include "corridormap/runtime.h"
#include "corridormap/build.h"

namespace corridormap {

namespace
{
    const float CORRIDORMAP_SQRT_2 = 1.41421356f;
    const float CORRIDORMAP_PI     = 3.14159265f;
}

Bbox2 bounds(const Footprint& f, float border)
{
    const float* x = f.x;
    const float* y = f.y;
    const int num_verts = f.num_verts;

    Bbox2 result;
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

float max_distance(Bbox2 bounds)
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

int max_distance_mesh_verts(const Footprint& f, float max_dist, float max_error)
{
    int point_tris = distance_mesh_tris_for_point(max_dist, max_error);
    // point_tris triangles per vertex, 2 triangles per edge, four border planes, obstacle polygons.
    return point_tris*f.num_verts*3 + f.num_verts*2*3 + (f.num_verts - f.num_polys*2)*3 + 6*4;
}

namespace
{
    inline int build_cone_sector(Render_Vertex*& output, Vec2 pos, int steps, float step_angle, float start_angle, float radius)
    {
        int nverts = 0;

        for (int i = 0; i < steps; ++i, nverts += 3)
        {
            Render_Vertex* a = output++;
            Render_Vertex* b = output++;
            Render_Vertex* c = output++;

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

    inline int build_tent_side(Render_Vertex*& output, Vec2 a, Vec2 b, float len, float size)
    {
        Vec2 e = sub(b, a);
        Vec2 n = scale(make_vec2(-e.y, e.x), 1.f/len);

        Render_Vertex p0 = { a.x, a.y, 0.f };
        Render_Vertex p1 = { b.x, b.y, 0.f };
        Render_Vertex p2 = { a.x + size*n.x, a.y + size*n.y, size };
        Render_Vertex p3 = { b.x + size*n.x, b.y + size*n.y, size };

        *output++ = p0; *output++ = p1; *output++ = p2;
        *output++ = p2; *output++ = p1; *output++ = p3;

        return 6;
    }

    inline int build_poly_cap(Render_Vertex*& output, const float* poly_x, const float* poly_y, int num_verts)
    {
        corridormap_assert(num_verts >= 3);
        Render_Vertex p0 = { poly_x[0], poly_y[0], 0.f };
        Render_Vertex p1 = { poly_x[1], poly_y[1], 0.f };

        for (int i = 2; i < num_verts; ++i)
        {
            Render_Vertex p2 = { poly_x[i], poly_y[i], 0.f };
            *output++ = p0;
            *output++ = p1;
            *output++ = p2;
            p1 = p2;
        }

        return (num_verts - 2)*3;
    }
}

void build_distance_mesh(const Footprint& in, Bbox2 bounds, float max_dist, float max_error, Distance_Mesh& out)
{
    corridormap_assert(max_dist > max_error);

    const float cone_half_angle = acos((max_dist-max_error) / max_dist);
    const int cone_triangle_count = static_cast<unsigned>(ceil(CORRIDORMAP_PI/cone_half_angle));
    const float cone_angle = 2.f*CORRIDORMAP_PI/cone_triangle_count;

    const int* num_poly_verts = in.num_poly_verts;
    const int num_polys = in.num_polys;

    // output
    Render_Vertex* verts = out.verts;
    unsigned int* segment_colors = out.segment_colors;
    int* num_segment_verts = out.num_segment_verts;

    int next_seg_color = 0;

    // 1. segment 0 is area inside obstacles.
    {
        const float* poly_x = in.x;
        const float* poly_y = in.y;
        int nsegverts = 0;

        for (int i = 0; i < num_polys; ++i)
        {
            int npverts = num_poly_verts[i];
            nsegverts += build_poly_cap(verts, poly_x, poly_y, npverts);
            poly_x += npverts;
            poly_y += npverts;
        }

        *segment_colors++ = next_seg_color++;
        *num_segment_verts++ = nsegverts;
    }

    const float* poly_x = in.x;
    const float* poly_y = in.y;

    for (int i = 0; i < num_polys; ++i)
    {
        int npverts = num_poly_verts[i];
        int nsegverts = 0;

        int prev_idx = npverts - 2;
        int curr_idx = npverts - 1;
        int next_idx = 0;

        for (; next_idx < npverts; prev_idx = curr_idx, curr_idx = next_idx++)
        {
            Vec2 prev = { poly_x[prev_idx], poly_y[prev_idx] };
            Vec2 curr = { poly_x[curr_idx], poly_y[curr_idx] };
            Vec2 next = { poly_x[next_idx], poly_y[next_idx] };

            float len_e1 = len(sub(next, curr));

            Vec2 e0 = normalized(sub(prev, curr));
            Vec2 e1 = normalized(sub(next, curr));

            float cos_inner = dot(e0, e1);
            float angle_inner = acos(cos_inner);
            float angle_cone_sector = 2.f * CORRIDORMAP_PI - angle_inner;

            int angle_cone_sector_steps = static_cast<unsigned>(ceil(angle_cone_sector/cone_angle));
            float angle_cone_sector_step = angle_cone_sector/angle_cone_sector_steps;
            float angle_start = atan2(e0.y, e0.x);

            // 2. generate cone sector for the current vertex.
            nsegverts += build_cone_sector(verts, curr, angle_cone_sector_steps, angle_cone_sector_step, angle_start, max_dist);

            // 3. generate tent for (curr, next) edge.
            nsegverts += build_tent_side(verts, next, curr, len_e1, max_dist);
        }

        poly_x += npverts;
        poly_y += npverts;

        *segment_colors++    = next_seg_color++;
        *num_segment_verts++ = nsegverts;
    }

    // 4. generate borders.
    {
        Vec2 lt = { bounds.min[0], bounds.max[1] };
        Vec2 lb = { bounds.min[0], bounds.min[1] };
        Vec2 rt = { bounds.max[0], bounds.max[1] };
        Vec2 rb = { bounds.max[0], bounds.min[1] };

        Vec2 len = sub(rt, lb);

        *num_segment_verts++ = build_tent_side(verts, lb, rb, len.x, max_dist);
        *num_segment_verts++ = build_tent_side(verts, rb, rt, len.y, max_dist);
        *num_segment_verts++ = build_tent_side(verts, rt, lt, len.x, max_dist);
        *num_segment_verts++ = build_tent_side(verts, lt, lb, len.y, max_dist);

        for (int i = 0; i < num_border_segments; ++i)
        {
            *segment_colors++ = next_seg_color++;
        }
    }

    out.num_segments = 1 + num_border_segments + in.num_polys;
    out.num_verts = static_cast<int>(verts - out.verts);
}

void render_distance_mesh(Renderer* render_iface, const Distance_Mesh& mesh)
{
    render_iface->begin();

    int vertices_offset = 0;

    for (int i = 0; i < mesh.num_segments; ++i)
    {
        int num_verts = mesh.num_segment_verts[i];
        unsigned color = mesh.segment_colors[i];
        const Render_Vertex* vertices = mesh.verts + vertices_offset;
        render_iface->draw(vertices, num_verts/3, color);
        vertices_offset += num_verts;
    }

    render_iface->end();
}

void set_segment_colors(Distance_Mesh& mesh, unsigned int* colors, int ncolors)
{
    for (int i = 0; i < mesh.num_segments; ++i)
    {
        mesh.segment_colors[i] = colors[i % ncolors];
    }
}

namespace
{
    inline void store_edge_normal(Vec2 u, Vec2 v, float*& out_x, float*& out_y)
    {
        Vec2 dir = normalized(sub(v, u));
        *out_x++ = +dir.y;
        *out_y++ = -dir.x;
    }
}

void build_footprint_normals(const Footprint& in, Bbox2 bounds, Footprint_Normals& out)
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
            Vec2 curr = { poly_x[curr_idx], poly_y[curr_idx] };
            Vec2 next = { poly_x[next_idx], poly_y[next_idx] };
            store_edge_normal(curr, next, normal_x, normal_y);
        }

        poly_x += nverts;
        poly_y += nverts;

        *num_obstacle_normals++ = nverts;
    }

    {
        Vec2 lt = { bounds.min[0], bounds.max[1] };
        Vec2 lb = { bounds.min[0], bounds.min[1] };
        Vec2 rt = { bounds.max[0], bounds.max[1] };
        Vec2 rb = { bounds.max[0], bounds.min[1] };

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
        const Vec2 edge_point)
    {
        int oid = obstacle_id - 1;

        if (oid < 0)
        {
            return 0;
        }

        int num_normals = num_obstacle_normals[oid];

        int first_normal_idx = obstacle_normal_offsets[oid];
        int last_normal_idx = first_normal_idx + num_normals - 1;

        int curr_idx = last_normal_idx;
        int next_idx = first_normal_idx;

        for (; next_idx <= last_normal_idx; curr_idx = next_idx++)
        {
            Vec2 vertex = { vertex_x[curr_idx], vertex_y[curr_idx] };
            Vec2 normal_curr = { normal_x[curr_idx], normal_y[curr_idx] };
            Vec2 normal_next = { normal_x[next_idx], normal_y[next_idx] };

            Vec2 mid = normalized(scale(add(normal_curr, normal_next), 0.5f));
            Vec2 dir = normalized(sub(edge_point, vertex));

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

void build_edge_point_normal_indices(const Voronoi_Features& features, const Footprint& obstacles, const Footprint_Normals& normals, Bbox2 bounds, Voronoi_Edge_Normals& out)
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

    int* normal_indices_1 = out.indices_1;
    int* normal_indices_2 = out.indices_2;

    float bounds_width = bounds.max[0] - bounds.min[0];
    float bounds_height = bounds.max[1] - bounds.min[1];

    for (int i = 0; i < num_edge_points; ++i)
    {
        unsigned int edge_point_idx = edges[i];
        unsigned int obstacle_1 = obstacle_ids_1[i];
        unsigned int obstacle_2 = obstacle_ids_2[i];

        int edge_point_x = edge_point_idx%grid_width;
        int edge_point_y = edge_point_idx/grid_width;

        Vec2 edge_point = { bounds.min[0] + float(edge_point_x)/grid_width * bounds_width, bounds.min[1] + float(edge_point_y)/grid_height * bounds_height};

        normal_indices_1[i] = find_normal_index(vertex_x, vertex_y, normal_x, normal_y, num_obstacle_normals, obstacle_normal_offsets, obstacle_1, edge_point);
        normal_indices_2[i] = find_normal_index(vertex_x, vertex_y, normal_x, normal_y, num_obstacle_normals, obstacle_normal_offsets, obstacle_2, edge_point);
    }
}

void build_csr(const unsigned int* nz_coords, CSR_Grid& out)
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

int nz(const CSR_Grid& grid, int row, int col)
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

int nz(const CSR_Grid& grid, int linear_index)
{
    return nz(grid, linear_index / grid.num_cols, linear_index % grid.num_cols);
}

namespace
{
    int nei_offset_row[] = { +0, -1, +1, +0 };
    int nei_offset_col[] = { -1, +0, +0, +1 };
}

CSR_Grid_Neis cell_neis(const CSR_Grid& grid, int row, int col)
{
    int num_rows = grid.num_rows;
    int num_cols = grid.num_cols;

    CSR_Grid_Neis neis;
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

CSR_Grid_Neis cell_neis(const CSR_Grid& grid, int linear_index)
{
    return cell_neis(grid, linear_index / grid.num_cols, linear_index % grid.num_cols);
}

namespace
{
    template <typename T>
    struct Queue
    {
        Queue(Memory* mem, int max_size)
            : front(0)
            , size(0)
            , max_size(max_size)
            , mem(mem)
        {
            data = allocate<T>(mem, max_size);
        }

        ~Queue()
        {
            mem->deallocate(data);
        }

        int front;
        int size;
        int max_size;
        Memory* mem;
        T* data;
    };

    template <typename T>
    int size(const Queue<T>& q)
    {
        return q.size;
    }

    template <typename T>
    void enqueue(Queue<T>& q, const T val)
    {
        int idx = (q.front + q.size) % q.max_size;
        q.data[idx] = val;
        q.size += 1;
    }

    template <typename T>
    T dequeue(Queue<T>& q)
    {
        T val = q.data[q.front % q.max_size];
        q.front++;
        q.size--;
        return val;
    }

    template <typename T>
    void clear(Queue<T>& q)
    {
        q.front = 0;
        q.size = 0;
    }

    int get_next_point(const CSR_Grid& edges, const Voronoi_Features& features, int current_point, int previous_point)
    {
        unsigned int side_1 = features.edge_obstacle_ids_1[nz(edges, current_point)];
        unsigned int side_2 = features.edge_obstacle_ids_2[nz(edges, current_point)];

        CSR_Grid_Neis neis = cell_neis(edges, current_point);

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

    int get_neigbour_vertex(const CSR_Grid& vertices, int cell_linear_index, int start_vert)
    {
        CSR_Grid_Neis neis = cell_neis(vertices, cell_linear_index);

        for (int i = 0; i < neis.num; ++i)
        {
            if (neis.lin_idx[i] != start_vert)
            {
                return neis.lin_idx[i];
            }
        }

        return -1;
    }

    void fix_sides(Voronoi_Features& features, Voronoi_Edge_Normals& normals, int prev_point_nz, int curr_point_nz)
    {
        unsigned int ps1 = features.edge_obstacle_ids_1[prev_point_nz];

        unsigned int cs1 = features.edge_obstacle_ids_1[curr_point_nz];
        unsigned int cs2 = features.edge_obstacle_ids_2[curr_point_nz];

        int ncs1 = normals.indices_1[curr_point_nz];
        int ncs2 = normals.indices_2[curr_point_nz];

        if (ps1 != cs1)
        {
            std::swap(cs1, cs2);
            std::swap(ncs1, ncs2);
        }

        features.edge_obstacle_ids_1[curr_point_nz] = cs1;
        features.edge_obstacle_ids_2[curr_point_nz] = cs2;

        normals.indices_1[curr_point_nz] = ncs1;
        normals.indices_2[curr_point_nz] = ncs2;
    }

    struct Traced_Incident_Edge
    {
        int vert;
        unsigned int color1;
        unsigned int color2;
    };

    Traced_Incident_Edge trace_incident_edge(const CSR_Grid& vertices, const CSR_Grid& edges, Voronoi_Features& features, Voronoi_Edge_Normals& normals,
                                             char* visited_edges, int start_vert, int edge_point)
    {
        Traced_Incident_Edge result;
        result.vert = -1;
        result.color1 = 0;
        result.color2 = 0;

        int prev = edge_point;

        for (int curr = edge_point; curr >= 0;)
        {
            int prev_nz = nz(edges, prev);
            int curr_nz = nz(edges, curr);

            if (visited_edges[curr_nz])
            {
                return result;
            }

            visited_edges[curr_nz] = 1;

            fix_sides(features, normals, prev_nz, curr_nz);

            int vert = get_neigbour_vertex(vertices, curr, start_vert);

            if (vert >= 0)
            {
                result.vert = vert;
                result.color1 = features.edge_obstacle_ids_1[curr_nz];
                result.color2 = features.edge_obstacle_ids_2[curr_nz];
                return result;
            }

            int next = get_next_point(edges, features, curr, prev);
            prev = curr;
            curr = next;
        }

        return result;
    }

    int trace_event_points(const CSR_Grid& vertices, const CSR_Grid& edges, const Voronoi_Features& features, const Voronoi_Edge_Normals& normals,
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

            int p_n1 = normals.indices_1[nz(edges, prev)];
            int p_n2 = normals.indices_2[nz(edges, prev)];
            int c_n1 = normals.indices_1[nz(edges, curr)];
            int c_n2 = normals.indices_2[nz(edges, curr)];

            if (p_n1 != c_n1 || p_n2 != c_n2)
            {
                if (p_n1 > 0 || p_n2 > 0)
                {
                    events[num_events] = prev;
                }
                else
                {
                    events[num_events] = curr;
                }

                num_events++;
            }

            int next = get_next_point(edges, features, curr, prev);
            prev = curr;
            curr = next;
        }

        return num_events;
    }
}

void trace_edges(Memory* scratch, const CSR_Grid& vertices, const CSR_Grid& edges,
                 Voronoi_Edge_Normals& edge_normal_indices, Voronoi_Features& features, Voronoi_Traced_Edges& out)
{
    Queue<int> queue_vert(scratch, vertices.num_nz);
    Alloc_Scope<char> visited_vert(scratch, vertices.num_nz);
    Alloc_Scope<char> visited_edge(scratch, edges.num_nz);
    zero_mem(visited_vert);
    zero_mem(visited_edge);

    int start_vert = features.verts[0];

    enqueue(queue_vert, start_vert);
    visited_vert[nz(vertices, start_vert)] = 1;

    int* out_u = out.u;
    int* out_v = out.v;
    unsigned int* out_obstacle_ids_1 = out.obstacle_ids_1;
    unsigned int* out_obstacle_ids_2 = out.obstacle_ids_2;
    int* out_events = out.events;
    int* out_event_offsets = out.edge_event_offset;
    int* out_num_events = out.edge_num_events;

    int num_edges = 0;
    int num_events = 0;

    while (size(queue_vert) > 0)
    {
        int u = dequeue(queue_vert);
        visited_vert[nz(vertices, u)] = 1;

        CSR_Grid_Neis neis = cell_neis(edges, u);

        for (int i = 0; i < neis.num; ++i)
        {
            Traced_Incident_Edge e = trace_incident_edge(vertices, edges, features, edge_normal_indices, visited_edge, u, neis.lin_idx[i]);

            if (e.vert < 0)
            {
                continue;
            }

            int v = e.vert;
            int num_edge_events = trace_event_points(vertices, edges, features, edge_normal_indices, u, neis.lin_idx[i], out_events + num_events);

            out_u[num_edges] = u;
            out_v[num_edges] = v;
            out_obstacle_ids_1[num_edges] = e.color1;
            out_obstacle_ids_2[num_edges] = e.color2;
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
    struct Segement_Closest_Point
    {
        Vec2 closest;
        float dist;
    };

    Segement_Closest_Point closest_to_segment(const Vec2& point, const Vec2& p0, const Vec2& p1)
    {
        Segement_Closest_Point result;

        Vec2 seg = sub(p1, p0);
        Vec2 dir = normalized(seg);
        float seg_len = len(seg);

        float proj = dot(sub(point, p0), dir);

        Vec2 seg_closest = add(p0, scale(dir, clamp(proj, 0.f, seg_len)));

        Vec2 seg_closest_dir = sub(seg_closest, point);
        float seg_closest_dist = dot(seg_closest_dir, seg_closest_dir);

        result.closest = seg_closest;
        result.dist = seg_closest_dist;

        return result;
    }

    Vec2 compute_closest_point(const Footprint& obstacles, const Bbox2& bounds, const int* obstacle_offsets, unsigned int obstacle_id, const Vec2& point)
    {
        int oid = obstacle_id - 1;

        if (oid < 0)
        {
            return point;
        }

        if (oid >= obstacles.num_polys)
        {
            Vec2 lt = { bounds.min[0], bounds.max[1] };
            Vec2 lb = { bounds.min[0], bounds.min[1] };
            Vec2 rt = { bounds.max[0], bounds.max[1] };
            Vec2 rb = { bounds.max[0], bounds.min[1] };

            Vec2 segments_p0[] = { lb, rb, rt, lt };
            Vec2 segments_p1[] = { rb, rt, lt, lb };

            int offset = oid - obstacles.num_polys;
            Segement_Closest_Point r = closest_to_segment(point, segments_p0[offset], segments_p1[offset]);
            return r.closest;
        }

        const float* obst_x = obstacles.x;
        const float* obst_y = obstacles.y;
        const int* num_obst_verts = obstacles.num_poly_verts;

        Vec2 closest = { FLT_MAX, FLT_MAX };
        float min_dist = FLT_MAX;

        int first_vertex_idx = obstacle_offsets[oid];
        int last_vertex_idx = first_vertex_idx + num_obst_verts[oid] - 1;

        int curr_idx = last_vertex_idx;
        int next_idx = first_vertex_idx;

        for (; next_idx <= last_vertex_idx; curr_idx = next_idx++)
        {
            Vec2 p0 = { obst_x[curr_idx], obst_y[curr_idx] };
            Vec2 p1 = { obst_x[next_idx], obst_y[next_idx] };

            Segement_Closest_Point r = closest_to_segment(point, p0, p1);

            if (r.dist < min_dist)
            {
                min_dist = r.dist;
                closest = r.closest;
            }
        }

        return closest;
    }

    Vec2 convert_from_image(int lin_idx, int grid_width, int grid_height, Bbox2 bounds)
    {
        float bounds_width = bounds.max[0] - bounds.min[0];
        float bounds_height = bounds.max[1] - bounds.min[1];
        float vx = float(lin_idx % grid_width);
        float vy = float(lin_idx / grid_width);
        vx = vx / grid_width * bounds_width + bounds.min[0];
        vy = vy / grid_height * bounds_height + bounds.min[1];
        return make_vec2(vx, vy);
    }

    bool is_left(Vec2 prev, Vec2 curr, Vec2 side_pos)
    {
        Vec2 dir1 = sub(curr, prev);
        Vec2 dir2 = sub(side_pos, prev);
        return dir1.x*dir2.y - dir1.y*dir2.x > 0.f;
    }
}

void build_walkable_space(const Footprint& obstacles, const int* obstacle_offsets, Bbox2 bounds, const Voronoi_Features& features,
                          const CSR_Grid& edge_grid,  const CSR_Grid& vertex_grid, const Voronoi_Traced_Edges& traced_edges, Walkable_Space& out)
{
    // 1. vertices: convert positions.
    for (int i = 0; i < features.num_vert_points; ++i)
    {
        Vec2 pos = convert_from_image(features.verts[i], features.grid_width, features.grid_height, bounds);
        create_vertex(out, pos);
    }

    // 2. create edges.
    for (int i = 0; i < traced_edges.num_edges; ++i)
    {
        int u_nz = nz(vertex_grid, traced_edges.u[i]);
        int v_nz = nz(vertex_grid, traced_edges.v[i]);

        Edge* edge = create_edge(out, u_nz, v_nz);
        Half_Edge* e0 = edge->dir + 0;
        Half_Edge* e1 = edge->dir + 1;

        Vec2 u = source(out, edge)->pos;
        Vec2 v = target(out, edge)->pos;

        unsigned int obstacle_id_1 = traced_edges.obstacle_ids_1[i];
        unsigned int obstacle_id_2 = traced_edges.obstacle_ids_2[i];

        {
            Vec2 cp0 = compute_closest_point(obstacles, bounds, obstacle_offsets, obstacle_id_1, target(out, e0)->pos);
            Vec2 cp1 = compute_closest_point(obstacles, bounds, obstacle_offsets, obstacle_id_2, target(out, e0)->pos);

            if (is_left(u, v, cp0))
            {
                e0->sides[0] = cp0;
                e0->sides[1] = cp1;
            }
            else
            {
                e0->sides[0] = cp1;
                e0->sides[1] = cp0;
            }
        }

        {
            Vec2 cp0 = compute_closest_point(obstacles, bounds, obstacle_offsets, obstacle_id_2, target(out, e1)->pos);
            Vec2 cp1 = compute_closest_point(obstacles, bounds, obstacle_offsets, obstacle_id_1, target(out, e1)->pos);

            if (is_left(v, u, cp0))
            {
                e1->sides[0] = cp0;
                e1->sides[1] = cp1;
            }
            else
            {
                e1->sides[0] = cp1;
                e1->sides[1] = cp0;
            }
        }
    }

    // 3. events: convert positions and compute sides.
    for (int i = 0; i < traced_edges.num_edges; ++i)
    {
        Edge* edge = out.edges.items + i;
        Vec2 u = source(out, edge)->pos;

        int event_offset = traced_edges.edge_event_offset[i];
        int num_events = traced_edges.edge_num_events[i];

        Vec2 prev = u;

        for (int j = event_offset; j < event_offset + num_events; ++j)
        {
            int evt_lin_idx = traced_edges.events[j];
            Vec2 pos = convert_from_image(evt_lin_idx, features.grid_width, features.grid_height, bounds);

            Event* e = create_event(out, pos, i);

            unsigned int obstacle_id_1 = features.edge_obstacle_ids_1[nz(edge_grid, evt_lin_idx)];
            unsigned int obstacle_id_2 = features.edge_obstacle_ids_2[nz(edge_grid, evt_lin_idx)];

            Vec2 cp0 = compute_closest_point(obstacles, bounds, obstacle_offsets, obstacle_id_1, pos);
            Vec2 cp1 = compute_closest_point(obstacles, bounds, obstacle_offsets, obstacle_id_2, pos);

            if (is_left(prev, pos, cp0))
            {
                e->sides[0] = cp0;
                e->sides[1] = cp1;
            }
            else
            {
                e->sides[0] = cp1;
                e->sides[1] = cp0;
            }

            prev = pos;
        }
    }
}

}
