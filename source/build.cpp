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

#include <math.h>
#include <float.h>
#include <string.h>
#include "corridormap/assert.h"
#include "corridormap/memory.h"
#include "corridormap/render_interface.h"
#include "corridormap/build.h"

namespace corridormap {

namespace
{
    const float CORRIDORMAP_SQRT_2 = 1.41421356f;
    const float CORRIDORMAP_PI     = 3.14159265f;

    // border gets a distance mesh segment (half tent) per side.
    enum { num_border_segments = 4 };
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
    result.num_segments = 0;
    result.num_verts = 0;
    int max_verts = max_distance_mesh_verts(f, max_dist, max_error);
    result.verts = allocate<vertex>(mem, max_verts * sizeof(vertex));
    // 4 segments for border and num_polys segments for obstacles.
    result.num_segment_verts = allocate<int>(mem, (num_border_segments+f.num_polys) * sizeof(int));
    result.segment_colors = allocate<unsigned int>(mem, (num_border_segments+f.num_polys) * sizeof(unsigned int));
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
    inline int build_cone_sector(vertex*& output, float pos[2], int steps, float step_angle, float start_angle, float radius)
    {
        int nverts = 0;

        for (int i = 0; i < steps; ++i, nverts += 3)
        {
            vertex* a = output++;
            vertex* b = output++;
            vertex* c = output++;

            a->x = pos[0];
            a->y = pos[1];
            a->z = 0.f;

            b->x = pos[0] + radius*cos(start_angle + (i+0) * step_angle);
            b->y = pos[1] + radius*sin(start_angle + (i+0) * step_angle);
            b->z = radius;

            c->x = pos[0] + radius*cos(start_angle + (i+1) * step_angle);
            c->y = pos[1] + radius*sin(start_angle + (i+1) * step_angle);
            c->z = radius;
        }

        return nverts;
    }

    inline int build_tent_side(vertex*& output, float a[2], float b[2], float len, float size)
    {
        float nx = -(b[1]-a[1]) / len;
        float ny = +(b[0]-a[0]) / len;

        vertex p0 = { a[0], a[1], 0.f };
        vertex p1 = { b[0], b[1], 0.f };
        vertex p2 = { a[0] + size*nx, a[1] + size*ny, size };
        vertex p3 = { b[0] + size*nx, b[1] + size*ny, size };

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
    vertex* verts = out.verts;
    unsigned int* segment_colors = out.segment_colors;
    int* num_segment_verts = out.num_segment_verts;

    // 1. generate borders.
    {
        float corner_lt[] = { bbox.min[0], bbox.max[1] };
        float corner_rb[] = { bbox.max[0], bbox.min[1] };

        float len_x = bbox.max[0] - bbox.min[0];
        float len_y = bbox.max[1] - bbox.min[1];

        out.num_segment_verts[0] = build_tent_side(verts, bbox.min, corner_rb, len_x, max_dist);
        out.num_segment_verts[1] = build_tent_side(verts, corner_rb, bbox.max, len_y, max_dist);
        out.num_segment_verts[2] = build_tent_side(verts, bbox.max, corner_lt, len_x, max_dist);
        out.num_segment_verts[3] = build_tent_side(verts, corner_lt, bbox.min, len_y, max_dist);

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
            float prev[] = { poly_x[prev_idx], poly_y[prev_idx] };
            float curr[] = { poly_x[curr_idx], poly_y[curr_idx] };
            float next[] = { poly_x[next_idx], poly_y[next_idx] };

            float e0[] = { prev[0] - curr[0], prev[1] - curr[1] };
            float e1[] = { next[0] - curr[0], next[1] - curr[1] };

            float len_e0 = sqrt(e0[0]*e0[0] + e0[1]*e0[1]);
            corridormap_assert(len_e0 > 0.f);

            float len_e1 = sqrt(e1[0]*e1[0] + e1[1]*e1[1]);
            corridormap_assert(len_e1 > 0.f);

            float cos_inner = (e0[0]*e1[0] + e0[1]*e1[1]) / (len_e0*len_e1);
            float angle_inner = acos(cos_inner);
            float angle_cone_sector = 2.f * CORRIDORMAP_PI - angle_inner;

            int angle_cone_sector_steps = static_cast<unsigned>(ceil(angle_cone_sector/cone_angle));
            float angle_cone_sector_step = angle_cone_sector/angle_cone_sector_steps;
            float angle_start = atan2(e0[1]/len_e0, e0[0]/len_e0);

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
        const vertex* vertices = mesh.verts + vertices_offset;
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

voronoi_features allocate_voronoi_features(memory* mem, int num_vert_points, int num_edge_points)
{
    voronoi_features result;
    memset(&result, 0, sizeof(result));

    result.num_vert_points = num_vert_points;
    result.num_edge_points = num_edge_points;
    result.verts = allocate<unsigned int>(mem, num_vert_points*sizeof(unsigned int));
    result.edges = allocate<unsigned int>(mem, num_edge_points*sizeof(unsigned int));
    result.vert_obstacle_ids = allocate<unsigned int>(mem, 4*num_vert_points*sizeof(unsigned int));
    result.edge_obstacle_ids_left = allocate<unsigned int>(mem, num_edge_points*sizeof(unsigned int));
    result.edge_obstacle_ids_right = allocate<unsigned int>(mem, num_edge_points*sizeof(unsigned int));
    result.edge_normal_indices_left = allocate<int>(mem, num_edge_points*sizeof(int));
    result.edge_normal_indices_right = allocate<int>(mem, num_edge_points*sizeof(int));

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
    result.x = allocate<float>(mem, num_normals*sizeof(float));
    result.y = allocate<float>(mem, num_normals*sizeof(float));
    result.num_poly_normals = allocate<int>(mem, num_polygons*sizeof(int));
    result.poly_normal_offsets = allocate<int>(mem, num_polygons*sizeof(int));

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
            float curr[] = { poly_x[curr_idx], poly_y[curr_idx] };
            float next[] = { poly_x[next_idx], poly_y[next_idx] };
            float edge[] = { next[0] - curr[0], next[1] - curr[1] };

            normal_x[num_normals] = +edge[1];
            normal_y[num_normals] = -edge[0];

            ++num_normals;
        }

        num_poly_normals[i] = nverts;
    }
}

}
