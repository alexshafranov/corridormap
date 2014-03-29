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
#include "corridormap/render_interface.h"
#include "corridormap/build.h"

namespace corridormap {

namespace
{
    const float CORRIDORMAP_SQRT_2 = 1.41421356f;
    const float CORRIDORMAP_PI     = 3.14159265f;
}

bbox2 bounds(const footprint& f)
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

    return result;
}

float max_distance(bbox2 scene_bbox)
{
    float w = scene_bbox.max[0] - scene_bbox.min[0];
    float h = scene_bbox.max[1] - scene_bbox.min[1];
    float s = (w > h) ? w : h;
    return s * CORRIDORMAP_SQRT_2;
}

int point_distance_mesh_tris(float max_dist, float max_error)
{
    float cone_half_angle = acos((max_dist - max_error)/max_dist);
    unsigned cone_triangle_count = static_cast<unsigned>(ceil(CORRIDORMAP_PI / cone_half_angle));
    return cone_triangle_count;
}

int max_distance_mesh_verts(const footprint& f, float max_dist, float max_error)
{
    int point_tris = point_distance_mesh_tris(max_dist, max_error);
    // point_tris triangles per vertex and 4 triangles per edge.
    return point_tris * f.num_verts * 3 + f.num_verts * 4 * 3;
}

distance_mesh allocate_distance_mesh(memory* mem, const footprint& f, float max_dist, float max_error)
{
    distance_mesh result;
    result.num_segments = 0;
    result.num_verts = 0;
    int max_verts = max_distance_mesh_verts(f, max_dist, max_error);
    result.verts = allocate<vertex>(mem, max_verts * sizeof(vertex));
    result.num_segment_verts = allocate<int>(mem, f.num_polys * sizeof(int));
    result.segment_colors = allocate<unsigned int>(mem, f.num_polys * sizeof(unsigned int));
    return result;
}

void deallocate_distance_mesh(memory* mem, distance_mesh& mesh)
{
    mem->deallocate(mesh.verts);
    mem->deallocate(mesh.num_segment_verts);
    mem->deallocate(mesh.segment_colors);
    memset(&mesh, 0, sizeof(distance_mesh));
}

void build_distance_mesh(const footprint& in, distance_mesh& out, float max_dist, float max_error)
{
    corridormap_assert(max_dist > max_error);

    const float cone_half_angle = acos((max_dist - max_error)/max_dist);
    const int cone_triangle_count = static_cast<unsigned>(ceil(CORRIDORMAP_PI / cone_half_angle));
    const float cone_angle = 2.f * CORRIDORMAP_PI / cone_triangle_count;

    int out_vertex_offset = 0;

    const float* poly_x = in.x;
    const float* poly_y = in.y;
    vertex* verts = out.verts;

    for (int i = 0; i < in.num_polys; ++i)
    {
        int num_poly_verts = in.num_poly_verts[i];
        int num_segment_verts = 0;

        int prev_idx = num_poly_verts - 2;
        int curr_idx = num_poly_verts - 1;
        int next_idx = 0;

        for (; next_idx < num_poly_verts; prev_idx = curr_idx, curr_idx = next_idx++)
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

            int angle_cone_sector_steps = static_cast<unsigned>(ceil(angle_cone_sector / cone_angle));
            float angle_cone_sector_step = angle_cone_sector / angle_cone_sector_steps;
            float angle_start = atan2(e0[1]/len_e0, e0[0]/len_e0);

            // 1. generate cone sectors for each vertex.

            vertex* cone = verts + out_vertex_offset;

            for (int k = 0; k < angle_cone_sector_steps; ++k)
            {
                vertex* a = cone + k*3 + 0;
                vertex* b = cone + k*3 + 1;
                vertex* c = cone + k*3 + 2;

                a->x = curr[0];
                a->y = curr[1];
                a->z = 0.f;

                b->x = curr[0] + max_dist * cos(angle_start + (k+0)*angle_cone_sector_step);
                b->y = curr[1] + max_dist * sin(angle_start + (k+0)*angle_cone_sector_step);
                b->z = max_dist;

                c->x = curr[0] + max_dist * cos(angle_start + (k+1)*angle_cone_sector_step);
                c->y = curr[1] + max_dist * sin(angle_start + (k+1)*angle_cone_sector_step);
                c->z = max_dist;
            }

            num_segment_verts += angle_cone_sector_steps*3;
            out_vertex_offset += angle_cone_sector_steps*3;

            // 2. generate tent for (curr, next) edge.

            vertex* tent = verts + out_vertex_offset;
            {
                float nx = max_dist * -e1[1]/len_e1;
                float ny = max_dist * +e1[0]/len_e1;

                vertex a = { curr[0], curr[1], 0.f };
                vertex b = { next[0], next[1], 0.f };
                vertex c = { curr[0] + nx, curr[1] + ny, max_dist };
                vertex d = { next[0] + nx, next[1] + ny, max_dist };
                vertex e = { curr[0] - nx, curr[1] - ny, max_dist };
                vertex f = { next[0] - nx, next[1] - ny, max_dist };

                tent[0]  = a; tent[1]  = b; tent[2]  = c;
                tent[3]  = c; tent[4]  = b; tent[5]  = d;
                tent[6]  = a; tent[7]  = e; tent[8]  = b;
                tent[9]  = b; tent[10] = e; tent[11] = f;
            }

            num_segment_verts += 12;
            out_vertex_offset += 12;
        }

        poly_x += num_poly_verts;
        poly_y += num_poly_verts;

        out.num_segment_verts[i] = num_segment_verts;
        out.segment_colors[i] = i + 1;
    }

    out.num_segments = in.num_polys;
    out.num_verts = out_vertex_offset;
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
        render_iface->draw(vertices, num_verts / 3, color);
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

}
