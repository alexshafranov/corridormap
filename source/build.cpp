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
#include <stdlib.h>
#include <math.h>
#include "corridormap/assert.h"
#include "corridormap/memory.h"
#include "corridormap/render_interface.h"
#include "corridormap/vec2.h"
#include "corridormap/runtime.h"
#include "corridormap/build_alloc.h"
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

Bbox2 fit(Bbox2 box, float aspect)
{
    float bounds_width = box.max[0] - box.min[0];
    float bounds_height = box.max[1] - box.min[1];

    if (bounds_width > bounds_height)
    {
        float new_height = bounds_width / aspect;
        float diff = new_height - bounds_height;
        box.max[1] += diff * 0.5f;
        box.min[1] -= diff * 0.5f;
    }
    else
    {
        float new_width = bounds_height * aspect;
        float diff = new_width - bounds_width;
        box.max[0] += diff * 0.5f;
        box.min[0] -= diff * 0.5f;
    }

    return box;
}

float max_distance(Bbox2 bounds)
{
    float w = bounds.max[0] - bounds.min[0];
    float h = bounds.max[1] - bounds.min[1];
    return std::max(w, h) * CORRIDORMAP_SQRT_2;
}

int distance_mesh_tris_for_point(float max_dist, float max_error)
{
    float cone_half_angle = acosf((max_dist-max_error)/max_dist);
    return unsigned(ceil(CORRIDORMAP_PI/cone_half_angle));
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

            b->x = pos.x + radius*cosf(start_angle + (i + 0)*step_angle);
            b->y = pos.y + radius*sinf(start_angle + (i + 0)*step_angle);
            b->z = radius;

            c->x = pos.x + radius*cosf(start_angle + (i + 1)*step_angle);
            c->y = pos.y + radius*sinf(start_angle + (i + 1)*step_angle);
            c->z = radius;
        }

        return nverts;
    }

    inline int build_tent_side(Render_Vertex*& output, Vec2 a, Vec2 b, float len, float size)
    {
        Vec2 e = b - a;
        Vec2 n = make_vec2(-e.y, e.x)/len;

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
    const int cone_triangle_count = unsigned(ceil(CORRIDORMAP_PI/cone_half_angle));
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

            float len_e1 = mag(next - curr);

            Vec2 e0 = normalized(prev - curr);
            Vec2 e1 = normalized(next - curr);

            float cos_inner = dot(e0, e1);
            float angle_inner = acos(cos_inner);
            float angle_cone_sector = 2.f * CORRIDORMAP_PI - angle_inner;

            int angle_cone_sector_steps = unsigned(ceil(angle_cone_sector/cone_angle));
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

        Vec2 len = rt - lb;

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
    out.num_verts = int(verts - out.verts);
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
    unsigned int pack_color(unsigned char* v)
    {
        return (unsigned int)(v[0]) << 24 |
               (unsigned int)(v[1]) << 16 |
               (unsigned int)(v[2]) << 8  |
               (unsigned int)(v[3]) << 0  ;
    }

    unsigned int get_pixel(unsigned char* data, int width, int x, int y)
    {
        return pack_color(data + (y*width*4 + x*4));
    }
}

Voronoi_Features detect_voronoi_features(Memory* memory, Memory* scratch, Renderer* render_iface)
{
    int width = render_iface->params.render_target_width;
    int height = render_iface->params.render_target_height;

    Alloc_Scope<unsigned char> colors(scratch, width*height*4);
    Alloc_Scope<unsigned char> output(scratch, width*height*1);
    corridormap_assert(colors.data);
    corridormap_assert(output.data);
    zero_mem(colors);
    zero_mem(output);

    render_iface->read_pixels(colors);

    int num_verts = 0;
    int num_edges = 0;

    for (int y = 1; y < height; ++y)
    {
        for (int x = 1; x < width; ++x)
        {
            unsigned int a = get_pixel(colors, width, x - 1, y - 1);
            unsigned int b = get_pixel(colors, width, x + 0, y - 1);
            unsigned int c = get_pixel(colors, width, x - 1, y + 0);
            unsigned int d = get_pixel(colors, width, x + 0, y + 0);

            int diff = 1;
            if (b != a) { diff++; }
            if (c != a && c != b) { diff++; }
            if (d != a && d != b && d != c) { diff++; }

            int num_zero = 0;
            if (a == 0) { num_zero++; }
            if (b == 0) { num_zero++; }
            if (c == 0) { num_zero++; }
            if (d == 0) { num_zero++; }

            unsigned char m = 0;

            if (diff > 2)
            {
                m |= 0x0f;
                num_verts++;
            }

            if (diff - num_zero == 2)
            {
                m |= 0xf0;
                num_edges++;
            }

            output[y*width + x] = m;
        }
    }

    Voronoi_Features features = allocate_voronoi_features(memory, width, height, num_verts, num_edges);
    memset(features.verts, 0, num_verts*sizeof(features.verts[0]));
    memset(features.edges, 0, num_edges*sizeof(features.edges[0]));

    int vert_top = 0;
    int edge_top = 0;

    for (int y = 1; y < height; ++y)
    {
        for (int x = 1; x < width; ++x)
        {
            unsigned int a = get_pixel(colors, width, x - 1, y - 1);
            unsigned int b = get_pixel(colors, width, x + 0, y - 1);
            unsigned int c = get_pixel(colors, width, x - 1, y + 0);
            unsigned int d = get_pixel(colors, width, x + 0, y + 0);

            int lin_idx = y*width + x;
            unsigned char m = output[lin_idx];

            // vertex
            if ((m & 0x0f) != 0)
            {
                features.verts[vert_top++] = lin_idx;
            }

            // edge
            if ((m & 0xf0) != 0)
            {
                features.edges[edge_top] = lin_idx;
                features.edge_obstacle_ids_1[edge_top] = a;
                features.edge_obstacle_ids_2[edge_top] = (a != b) ? b : ((a != c) ? c : d);
                edge_top++;
            }
        }
    }

    return features;
}

namespace
{
    inline void store_edge_normal(Vec2 u, Vec2 v, float*& out_x, float*& out_y)
    {
        Vec2 dir = normalized(v - u);
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

            Vec2 mid = normalized((normal_curr + normal_next)*0.5f);
            Vec2 dir = normalized(edge_point - vertex);

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

void build_edge_spans(const Voronoi_Features& features, const Footprint& obstacles, const Footprint_Normals& normals, Bbox2 bounds, Voronoi_Edge_Spans& out)
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

    void fix_sides(Voronoi_Features& features, Voronoi_Edge_Spans& spans, int prev_point_nz, int curr_point_nz)
    {
        unsigned int ps1 = features.edge_obstacle_ids_1[prev_point_nz];

        unsigned int cs1 = features.edge_obstacle_ids_1[curr_point_nz];
        unsigned int cs2 = features.edge_obstacle_ids_2[curr_point_nz];

        int ncs1 = spans.indices_1[curr_point_nz];
        int ncs2 = spans.indices_2[curr_point_nz];

        if (ps1 != cs1)
        {
            std::swap(cs1, cs2);
            std::swap(ncs1, ncs2);
        }

        features.edge_obstacle_ids_1[curr_point_nz] = cs1;
        features.edge_obstacle_ids_2[curr_point_nz] = cs2;

        spans.indices_1[curr_point_nz] = ncs1;
        spans.indices_2[curr_point_nz] = ncs2;
    }

    struct Traced_Incident_Edge
    {
        int vert;
        unsigned int color1;
        unsigned int color2;
    };

    Traced_Incident_Edge trace_incident_edge(const CSR_Grid& vertices, const CSR_Grid& edges, Voronoi_Features& features, Voronoi_Edge_Spans& spans,
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

            fix_sides(features, spans, prev_nz, curr_nz);

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

    int trace_event_points(const CSR_Grid& vertices, const CSR_Grid& edges, const Voronoi_Features& features, const Voronoi_Edge_Spans& spans,
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

            int p_n1 = spans.indices_1[nz(edges, prev)];
            int p_n2 = spans.indices_2[nz(edges, prev)];
            int c_n1 = spans.indices_1[nz(edges, curr)];
            int c_n2 = spans.indices_2[nz(edges, curr)];

            // side 1 event.
            if (p_n1 != c_n1)
            {
                events[num_events++] = c_n1 > 0 ? curr : prev;
            }
            // side 2 event.
            else if (p_n2 != c_n2)
            {
                events[num_events++] = c_n2 > 0 ? -curr : -prev;
            }

            int next = get_next_point(edges, features, curr, prev);
            prev = curr;
            curr = next;
        }

        return num_events;
    }
}

void trace_edges(Memory* scratch, const CSR_Grid& vertices, const CSR_Grid& edges,
                 Voronoi_Edge_Spans& spans, Voronoi_Features& features, Voronoi_Traced_Edges& out)
{
    Dequeue<int> queue_vert(scratch, vertices.num_nz);
    Alloc_Scope<char> visited_vert(scratch, vertices.num_nz);
    Alloc_Scope<char> visited_edge(scratch, edges.num_nz);
    zero_mem(visited_vert);
    zero_mem(visited_edge);

    int start_vert = features.verts[0];

    push_back(queue_vert, start_vert);
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
        int u = pop_front(queue_vert);
        visited_vert[nz(vertices, u)] = 1;

        CSR_Grid_Neis neis = cell_neis(edges, u);

        for (int i = 0; i < neis.num; ++i)
        {
            Traced_Incident_Edge e = trace_incident_edge(vertices, edges, features, spans, visited_edge, u, neis.lin_idx[i]);

            if (e.vert < 0)
            {
                continue;
            }

            int v = e.vert;
            int num_edge_events = trace_event_points(vertices, edges, features, spans, u, neis.lin_idx[i], out_events + num_events);

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
                push_back(queue_vert, v);
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

        Vec2 seg = p1 - p0;
        Vec2 dir = normalized(seg);
        float seg_len = mag(seg);

        float proj = dot(point - p0, dir);

        Vec2 seg_closest = p0 + dir*clamp(proj, 0.f, seg_len);

        Vec2 seg_closest_dir = seg_closest - point;
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
        Vec2 dir1 = curr - prev;
        Vec2 dir2 = side_pos - prev;
        return det(dir1, dir2) > 0.f;
    }

    struct Event_Closest_Points
    {
        Vec2 pos;
        Vec2 cp1;
        Vec2 cp2;
    };

    // correct sampled event position so that the point lies on the corresponding obstacle vertex normal.
    Event_Closest_Points correct_pos_and_compute_closest(int event, int event_nz_index, Vec2 sampled_pos, Bbox2 bounds,
                                                         const Footprint* obstacles, const Footprint_Normals* obstacle_normals,
                                                         const Voronoi_Edge_Spans* spans, const Voronoi_Features* features)
    {
        Event_Closest_Points result;
        int* obstacle_offsets = obstacle_normals->obstacle_normal_offsets;

        int vertex_index_1 = spans->indices_1[event_nz_index];
        int vertex_index_2 = spans->indices_2[event_nz_index];
        int obstacle_id_1 = features->edge_obstacle_ids_1[event_nz_index];
        int obstacle_id_2 = features->edge_obstacle_ids_2[event_nz_index];

        int vertex_index = event > 0 ? vertex_index_1 : vertex_index_2;
        int obstacle_id = event > 0 ? obstacle_id_1 : obstacle_id_2;

        corridormap_assert(vertex_index > 0);
        vertex_index = vertex_index - 1;

        if (vertex_index >= obstacles->num_verts)
        {
            result.pos = sampled_pos;
            result.cp1 = compute_closest_point(*obstacles, bounds, obstacle_offsets, obstacle_id_1, sampled_pos);
            result.cp2 = compute_closest_point(*obstacles, bounds, obstacle_offsets, obstacle_id_2, sampled_pos);
            return result;
        }

        // get two normals associated with vertex and project event point to the closest normal.
        int num_normals = obstacle_normals->num_obstacle_normals[obstacle_id - 1];
        int first_normal_idx = obstacle_normals->obstacle_normal_offsets[obstacle_id - 1];
        int curr = vertex_index;
        int next = first_normal_idx + (curr - first_normal_idx + 1) % num_normals;
        Vec2 v = { obstacles->x[vertex_index], obstacles->y[vertex_index] };
        Vec2 n1 = { obstacle_normals->x[curr], obstacle_normals->y[curr] };
        Vec2 n2 = { obstacle_normals->x[next], obstacle_normals->y[next] };
        Vec2 dir = sampled_pos - v;
        float p1 = dot(dir, n1);
        float p2 = dot(dir, n2);
        result.pos = v + ((p1 > p2) ? n1*p1 : n2*p2);

        if (event > 0)
        {
            result.cp1 = v;
            result.cp2 = compute_closest_point(*obstacles, bounds, obstacle_offsets, obstacle_id_2, result.pos);
        }
        else
        {
            result.cp2 = v;
            result.cp1 = compute_closest_point(*obstacles, bounds, obstacle_offsets, obstacle_id_1, result.pos);
        }

        return result;
    }

    void create_vertices(const Walkable_Space_Build_Params& in, Walkable_Space& out)
    {
        for (int i = 0; i < in.features->num_vert_points; ++i)
        {
            Vec2 pos = convert_from_image(in.features->verts[i], in.features->grid_width, in.features->grid_height, in.bounds);
            create_vertex(out, pos);
        }
    }

    void create_edges(const Walkable_Space_Build_Params& in, Walkable_Space& out)
    {
        for (int i = 0; i < in.traced_edges->num_edges; ++i)
        {
            int u_nz = nz(*in.vertex_grid, in.traced_edges->u[i]);
            int v_nz = nz(*in.vertex_grid, in.traced_edges->v[i]);
            create_edge(out, u_nz, v_nz);
        }
    }

    void create_events(const Walkable_Space_Build_Params& in, Walkable_Space& out)
    {
        for (int i = 0; i < in.traced_edges->num_edges; ++i)
        {
            Edge* edge = out.edges.items + i;
            Vec2 u = source(out, edge)->pos;

            int event_offset = in.traced_edges->edge_event_offset[i];
            int num_events = in.traced_edges->edge_num_events[i];

            Vec2 prev = u;

            for (int j = event_offset; j < event_offset + num_events; ++j)
            {
                int evt = in.traced_edges->events[j];
                int evt_lin_idx = ::abs(evt);
                int evt_nz_index = nz(*in.edge_grid, evt_lin_idx);

                Vec2 sampled_pos = convert_from_image(evt_lin_idx, in.features->grid_width, in.features->grid_height, in.bounds);
                Event_Closest_Points r = correct_pos_and_compute_closest(evt, evt_nz_index, sampled_pos, in.bounds,
                                                                         in.obstacles, in.obstacle_normals, in.spans, in.features);

                Event* e = create_event(out, r.pos, i);

                if (is_left(prev, r.pos, r.cp1))
                {
                    e->sides[0] = r.cp1;
                    e->sides[1] = r.cp2;
                }
                else
                {
                    e->sides[0] = r.cp2;
                    e->sides[1] = r.cp1;
                }

                prev = r.pos;
            }
        }
    }

    void compute_vertex_closest_points(const Walkable_Space_Build_Params& in, Walkable_Space& out)
    {
        int* obstacle_offsets = in.obstacle_normals->obstacle_normal_offsets;

        for (int i = 0; i < in.traced_edges->num_edges; ++i)
        {
            Edge* edge = out.edges.items + i;
            Half_Edge* e0 = edge->dir + 0;
            Half_Edge* e1 = edge->dir + 1;

            Vec2 u = source(out, edge)->pos;
            Vec2 v = target(out, edge)->pos;

            Vec2 u_prev = v;
            if (event(out, e0))
            {
                u_prev = event(out, e0)->pos;
            }

            Vec2 v_prev = u;
            if (event(out, e1))
            {
                v_prev = event(out, e1)->pos;
            }

            unsigned int obstacle_id_1 = in.traced_edges->obstacle_ids_1[i];
            unsigned int obstacle_id_2 = in.traced_edges->obstacle_ids_2[i];
            Vec2 cp01 = compute_closest_point(*in.obstacles, in.bounds, obstacle_offsets, obstacle_id_1, target(out, e0)->pos);
            Vec2 cp02 = compute_closest_point(*in.obstacles, in.bounds, obstacle_offsets, obstacle_id_2, target(out, e0)->pos);
            Vec2 cp11 = compute_closest_point(*in.obstacles, in.bounds, obstacle_offsets, obstacle_id_2, target(out, e1)->pos);
            Vec2 cp12 = compute_closest_point(*in.obstacles, in.bounds, obstacle_offsets, obstacle_id_1, target(out, e1)->pos);

            if (is_left(v_prev, v, cp01))
            {
                e0->sides[0] = cp01;
                e0->sides[1] = cp02;
            }
            else
            {
                e0->sides[0] = cp02;
                e0->sides[1] = cp01;
            }

            if (is_left(u_prev, u, cp11))
            {
                e1->sides[0] = cp11;
                e1->sides[1] = cp12;
            }
            else
            {
                e1->sides[0] = cp12;
                e1->sides[1] = cp11;
            }
        }
    }

    void prune_dead_ends(Walkable_Space& out)
    {
        for (Edge* e = first(out.edges); e != 0;)
        {
            Edge* next_e = next(out.edges, e);

            Vertex* s = source(out, e);
            Vertex* t = target(out, e);

            int ds = degree(out, s);
            int dt = degree(out, t);

            Vertex* v = (ds == 1) ? s : t;
            Vertex* u = (ds == 1) ? t : s;
            // half-edge: u->v
            Half_Edge* uv = (ds == 1) ? e->dir+1 : e->dir+0;

            if (ds == 1 || dt == 1)
            {
                // edge ending with degree one vertex, no events -> prune.
                if (!event(out, e->dir+0))
                {
                    deallocate(out.vertices, v);

                    for (Half_Edge* n = next(out, uv); ; n = next(out, n))
                    {
                        if (next(out, n) == uv)
                        {
                            n->next = uv->next;

                            if (half_edge(out, u) == uv)
                            {
                                Edge* n_edge = edge(out, n);
                                u->half_edge = int(n_edge - out.edges.items) + int(n - n_edge->dir);
                            }

                            break;
                        }
                    }

                    deallocate(out.edges, e);
                }
                // otherwise move the degree one vertex to the first event on that edge.
                else
                {
                    // ourgoing half-edge from v (degree one vertex).
                    Half_Edge* vu = opposite(out, uv);
                    Event* evt = event(out, vu);
                    Event* next_evt = next(out, vu, evt);
                    int vu_dir = int(vu - e->dir);

                    if (next_evt)
                    {
                        vu->event = int(next_evt - out.events.items);
                        next_evt->next[vu_dir^1] = null_idx;
                    }
                    else
                    {
                        vu->event = null_idx;
                        uv->event = null_idx;
                    }

                    v->pos = evt->pos;
                    uv->sides[0] = left_side(out, uv, evt);
                    uv->sides[1] = right_side(out, uv, evt);

                    deallocate(out.events, evt);
                }
            }

            e = next_e;
        }
    }

    void prune_disconnected_verts(Walkable_Space& out)
    {
        for (Vertex* v = first(out.vertices); v != 0;)
        {
            Vertex* next_v = next(out.vertices, v);

            if (!half_edge(out, v))
            {
                deallocate(out.vertices, v);
            }

            v = next_v;
        }
    }
}

void build_walkable_space(const Walkable_Space_Build_Params& in, Walkable_Space& out)
{
    create_vertices(in, out);
    create_edges(in, out);
    create_events(in, out);
    compute_vertex_closest_points(in, out);
    prune_dead_ends(out);
    prune_disconnected_verts(out);
}

}
