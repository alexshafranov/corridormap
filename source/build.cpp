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

namespace corridormap {

namespace
{
    const float CORRIDORMAP_SQRT_2 = 1.41421356f;
    const float CORRIDORMAP_PI     = 3.14159265f;

    // border gets a distance mesh segment (half tent) per side.
    enum { num_border_segments = 4 };

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
    bbox2 result;
    result.min[0] = +FLT_MAX;
    result.min[1] = +FLT_MAX;
    result.max[0] = -FLT_MAX;
    result.max[1] = -FLT_MAX;

    for (int i = 0; i < f.num_verts; ++i)
    {
        result.min[0] = f.x[i] < result.min[0] ? f.x[i] : result.min[0];
        result.min[1] = f.y[i] < result.min[1] ? f.y[i] : result.min[1];
        result.max[0] = f.x[i] > result.max[0] ? f.x[i] : result.max[0];
        result.max[1] = f.y[i] > result.max[1] ? f.y[i] : result.max[1];
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
    float s = (w > h) ? w : h;
    return s * CORRIDORMAP_SQRT_2;
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

distance_mesh allocate_distance_mesh(memory* mem, const footprint& f, float max_dist, float max_error)
{
    distance_mesh result;
    memset(&result, 0, sizeof(result));

    result.num_segments = 0;
    result.num_verts = 0;
    int max_verts = max_distance_mesh_verts(f, max_dist, max_error);
    result.verts = allocate<render_vertex>(mem, max_verts);
    // 4 segments for border and num_polys segments for obstacles.
    result.num_segment_verts = allocate<int>(mem, num_border_segments + f.num_polys);
    result.segment_colors = allocate<unsigned int>(mem, num_border_segments + f.num_polys);
    return result;
}

void deallocate_distance_mesh(memory* mem, distance_mesh& mesh)
{
    mem->deallocate(mesh.verts);
    mem->deallocate(mesh.num_segment_verts);
    mem->deallocate(mesh.segment_colors);
    memset(&mesh, 0, sizeof(mesh));
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

    // 1. generate borders.
    {
        vec2 lt = { bbox.min[0], bbox.max[1] };
        vec2 lb = { bbox.min[0], bbox.min[1] };
        vec2 rt = { bbox.max[0], bbox.max[1] };
        vec2 rb = { bbox.max[0], bbox.min[1] };

        vec2 len = sub(rt, lb);

        out.num_segment_verts[0] = build_tent_side(verts, lb, rb, len.x, max_dist);
        out.num_segment_verts[1] = build_tent_side(verts, rb, rt, len.y, max_dist);
        out.num_segment_verts[2] = build_tent_side(verts, rt, lt, len.x, max_dist);
        out.num_segment_verts[3] = build_tent_side(verts, lt, lb, len.y, max_dist);

        for (int i = 0; i < num_border_segments; ++i)
        {
            segment_colors[i] = i;
        }
    }

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

            // 2. generate cone sector for the current vertex.
            nsegverts += build_cone_sector(verts, curr, angle_cone_sector_steps, angle_cone_sector_step, angle_start, max_dist);

            // 3. generate tent for (curr, next) edge.
            nsegverts += build_tent_side(verts, curr, next, len_e1, max_dist);
            nsegverts += build_tent_side(verts, next, curr, len_e1, max_dist);
        }

        poly_x += npverts;
        poly_y += npverts;

        num_segment_verts[i + num_border_segments] = nsegverts;
        segment_colors[i] = i + num_border_segments;
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

voronoi_features allocate_voronoi_features(memory* mem, int grid_width, int num_vert_points, int num_edge_points)
{
    voronoi_features result;
    memset(&result, 0, sizeof(result));

    result.grid_width = grid_width;
    result.num_vert_points = num_vert_points;
    result.num_edge_points = num_edge_points;
    result.verts = allocate<unsigned int>(mem, num_vert_points);
    result.edges = allocate<unsigned int>(mem, num_edge_points);
    result.vert_obstacle_ids = allocate<unsigned int>(mem, 4*num_vert_points);
    result.edge_obstacle_ids_left = allocate<unsigned int>(mem, num_edge_points);
    result.edge_obstacle_ids_right = allocate<unsigned int>(mem, num_edge_points);

    return result;
}

void deallocate_voronoi_features(memory* mem, voronoi_features& features)
{
    mem->deallocate(features.verts);
    mem->deallocate(features.edges);
    mem->deallocate(features.vert_obstacle_ids);
    mem->deallocate(features.edge_obstacle_ids_left);
    mem->deallocate(features.edge_obstacle_ids_right);
    memset(&features, 0, sizeof(features));
}

footprint_normals allocate_foorprint_normals(memory* mem, int num_polygons, int num_normals)
{
    footprint_normals result;
    memset(&result, 0, sizeof(result));

    result.num_polys = num_polygons;
    result.num_normals = num_normals;
    result.x = allocate<float>(mem, num_normals);
    result.y = allocate<float>(mem, num_normals);
    result.num_poly_normals = allocate<int>(mem, num_polygons);
    result.poly_normal_offsets = allocate<int>(mem, num_polygons);

    return result;
}

// deallocates footprint normals. 'mem' must be the same that was used for allocation.
void deallocate_foorprint_normals(memory* mem, footprint_normals& normals)
{
    mem->deallocate(normals.x);
    mem->deallocate(normals.y);
    mem->deallocate(normals.num_poly_normals);
    mem->deallocate(normals.poly_normal_offsets);
    memset(&normals, 0, sizeof(normals));
}

void build_footprint_normals(const footprint& in, footprint_normals& out)
{
    const float* poly_x = in.x;
    const float* poly_y = in.y;
    const int* num_poly_verts = in.num_poly_verts;
    const int num_polys = in.num_polys;

    float* normal_x = out.x;
    float* normal_y = out.y;
    int* num_poly_normals = out.num_poly_normals;
    int* poly_normal_offsets = out.poly_normal_offsets;

    int num_normals = 0;

    for (int i = 0; i < num_polys; ++i)
    {
        poly_normal_offsets[i] = num_normals;

        int nverts = num_poly_verts[i];

        int curr_idx = nverts - 1;
        int next_idx = 0;

        for (; next_idx < nverts; curr_idx = next_idx++)
        {
            vec2 curr = { poly_x[curr_idx], poly_y[curr_idx] };
            vec2 next = { poly_x[next_idx], poly_y[next_idx] };

            vec2 dir = normalized(sub(next, curr));

            normal_x[num_normals] = +dir.y;
            normal_y[num_normals] = -dir.x;

            ++num_normals;
        }

        num_poly_normals[i] = nverts;
    }
}

voronoi_edge_normals allocate_voronoi_edge_normals(memory* mem, int num_edge_points)
{
    voronoi_edge_normals result;
    memset(&result, 0, sizeof(result));
    result.edge_normal_indices_left = allocate<int>(mem, num_edge_points);
    result.edge_normal_indices_right = allocate<int>(mem, num_edge_points);
    return result;
}

void deallocate_voronoi_edge_normals(memory* mem, voronoi_edge_normals& result)
{
    mem->deallocate(result.edge_normal_indices_left);
    mem->deallocate(result.edge_normal_indices_right);
    memset(&result, 0, sizeof(result));
}

namespace
{
    int find_normal_index(
        const float* vertex_x,
        const float* vertex_y,
        const float* normal_x,
        const float* normal_y,
        const int* num_poly_normals,
        const int* poly_normal_offsets,
        const int obstacle_id,
        const vec2 edge_point)
    {
        int num_normals = num_poly_normals[obstacle_id];
        int first_normal_idx = poly_normal_offsets[obstacle_id];
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
    const int* num_poly_normals = normals.num_poly_normals;
    const int* poly_normal_offsets = normals.poly_normal_offsets;

    int* normal_indices_left = out.edge_normal_indices_left;
    int* normal_indices_right = out.edge_normal_indices_right;

    for (int i = 0; i < num_edge_points; ++i)
    {
        unsigned int edge_point_idx = edges[i];
        unsigned int obstacle_left = obstacle_ids_left[i];
        unsigned int obstacle_right = obstacle_ids_right[i];

        vec2 edge_point = { float(edge_point_idx%grid_width), float(edge_point_idx/grid_width) };

        int left_idx  = find_normal_index(vertex_x, vertex_y, normal_x, normal_y, num_poly_normals, poly_normal_offsets, obstacle_left, edge_point);
        int right_idx = find_normal_index(vertex_x, vertex_y, normal_x, normal_y, num_poly_normals, poly_normal_offsets, obstacle_right, edge_point);

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

}
