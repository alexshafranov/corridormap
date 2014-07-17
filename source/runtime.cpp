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
#include <string.h>
#include <float.h>
#include <math.h>
#include "corridormap/assert.h"
#include "corridormap/memory.h"
#include "corridormap/vec2.h"
#include "corridormap/runtime.h"

namespace corridormap {

namespace
{
    const float CORRIDORMAP_PI = 3.14159265f;
}

Walkable_Space create_walkable_space(Memory* mem, int max_vertices, int max_edges, int max_events)
{
    Walkable_Space result;
    memset(&result, 0, sizeof(result));

    result.vertices.items = allocate<Vertex>(mem, max_vertices);
    result.edges.items = allocate<Edge>(mem, max_edges);
    result.events.items = allocate<Event>(mem, max_events);

    result.vertices.max_items = max_vertices;
    result.edges.max_items = max_edges;
    result.events.max_items = max_events;

    init(result.vertices);
    init(result.edges);
    init(result.events);

    return result;
}

void destroy(Memory* mem, Walkable_Space& d)
{
    mem->deallocate(d.vertices.items);
    mem->deallocate(d.edges.items);
    mem->deallocate(d.events.items);
    memset(&d, 0, sizeof(d));
}

namespace
{
    bool is_ccw(Vec2 u, Vec2 v1, Vec2 v2)
    {
        Vec2 d1 = sub(v1, u);
        Vec2 d2 = sub(v2, u);
        return det(d1, d2) > 0.f;
    }

    Half_Edge* get_half_edge(Edge* edges, int idx)
    {
        return edges[idx>>1].dir + (idx&1);
    }

    void add_half_edge(Vertex* vertices, Edge* edges, int vert, int h_edge)
    {
        int head = vertices[vert].half_edge;
        Half_Edge* new_half_edge = get_half_edge(edges, h_edge);

        if (head == null_idx)
        {
            vertices[vert].half_edge = h_edge;
            new_half_edge->next = h_edge;
        }
        else
        {
            Vec2 u = vertices[vert].pos;
            Vec2 v2 = vertices[new_half_edge->target].pos;

            int insert_after = null_idx;
            int curr = head;

            for (;;)
            {
                Half_Edge* curr_half_edge = get_half_edge(edges, curr);

                Vec2 v1 = vertices[curr_half_edge->target].pos;

                if (!is_ccw(u, v1, v2))
                {
                    break;
                }

                insert_after = curr;
                curr = curr_half_edge->next;

                if (curr == head)
                {
                    break;
                }
            }

            if (insert_after == null_idx)
            {
                Half_Edge* tail_half_edge = get_half_edge(edges, head);

                while (tail_half_edge->next != head)
                {
                    tail_half_edge = get_half_edge(edges, tail_half_edge->next);
                }

                vertices[vert].half_edge = h_edge;
                new_half_edge->next = head;
                tail_half_edge->next = h_edge;
            }
            else
            {
                Half_Edge* insert_after_half_edge = get_half_edge(edges, insert_after);
                int next = insert_after_half_edge->next;
                insert_after_half_edge->next = h_edge;
                new_half_edge->next = next;
            }
        }
    }

    void append_event(Edge* edges, Event* events, int h_edge, int evt)
    {
        int dir = h_edge & 1;
        Half_Edge* h = get_half_edge(edges, h_edge);
        int head = h->event;

        if (head == null_idx)
        {
            h->event = evt;
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

    void prepend_event(Edge* edges, Event* events, int h_edge, int evt)
    {
        int dir = h_edge & 1;
        Half_Edge* h = get_half_edge(edges, h_edge);
        int head = h->event;
        events[evt].next[dir] = head;
        h->event = evt;
    }
}

Vertex* create_vertex(Walkable_Space& space, Vec2 pos)
{
    Vertex* new_vertex = allocate(space.vertices);
    new_vertex->half_edge = null_idx;
    new_vertex->pos = pos;
    return new_vertex;
}

Edge* create_edge(Walkable_Space& space, int u, int v)
{
    Edge* new_edge = allocate(space.edges);
    new_edge->dir[0].target = v;
    new_edge->dir[1].target = u;
    new_edge->dir[0].event = null_idx;
    new_edge->dir[1].event = null_idx;
    int new_edge_idx = int(new_edge - space.edges.items);
    add_half_edge(space.vertices.items, space.edges.items, u, new_edge_idx*2 + 0);
    add_half_edge(space.vertices.items, space.edges.items, v, new_edge_idx*2 + 1);
    return new_edge;
}

Event* create_event(Walkable_Space& space, Vec2 pos, int edge)
{
    Event* new_event = allocate(space.events);
    new_event->pos = pos;
    new_event->next[0] = null_idx;
    new_event->next[1] = null_idx;
    int new_event_idx = int(new_event - space.events.items);
    append_event(space.edges.items, space.events.items, edge*2 + 0, new_event_idx);
    prepend_event(space.edges.items, space.events.items, edge*2 + 1, new_event_idx);
    return new_event;
}

int degree(const Walkable_Space& space, const Vertex* vertex)
{
    int result = 0;
    Half_Edge* head = half_edge(space, vertex);

    for (Half_Edge* edge = head; ; edge = next(space, edge))
    {
        result++;

        if (next(space, edge) == head)
        {
            break;
        }
    }

    return result;
}

int num_path_discs(const Walkable_Space& space, Half_Edge** path, int path_size)
{
    int result = 0;

    for (int i = 0; i < path_size; ++i)
    {
        const Half_Edge* e = path[i];
        // assert subsequent edges in the path share a vertex.
        corridormap_assert((i + 1 == path_size) || (e->target == opposite(space, path[i+1])->target));
        // count events.
        for (Event* evt = event(space, e); evt != 0; evt = next(space, e, evt))
        {
            result++;
        }
        // add two vertices per edge.
        result += 2;
    }

    return result;
}

Corridor create_corridor(Memory* mem, int max_disks, int max_portals)
{
    Corridor result;
    memset(&result, 0, sizeof(result));
    result.origin = allocate<Vec2>(mem, max_disks);
    result.radius = allocate<float>(mem, max_disks);
    result.obstacle_l = allocate<Vec2>(mem, max_disks);
    result.obstacle_r = allocate<Vec2>(mem, max_disks);
    result.border_l = allocate<Vec2>(mem, max_disks);
    result.border_r = allocate<Vec2>(mem, max_disks);
    result.curves = allocate<unsigned char>(mem, max_disks);
    result.portal_l = allocate<Vec2>(mem, max_portals);
    result.portal_r = allocate<Vec2>(mem, max_portals);
    result.max_portals = max_portals;
    result.max_disks = max_disks;
    return result;
}

void destroy(Memory* mem, Corridor& c)
{
    mem->deallocate(c.origin);
    mem->deallocate(c.radius);
    mem->deallocate(c.obstacle_l);
    mem->deallocate(c.obstacle_r);
    mem->deallocate(c.border_l);
    mem->deallocate(c.border_r);
    mem->deallocate(c.curves);
    mem->deallocate(c.portal_l);
    mem->deallocate(c.portal_r);
    memset(&c, 0, sizeof(c));
}

void vertex_to_edge_path(const Walkable_Space& space, Vertex** path, int path_size, Half_Edge** out)
{
    for (int i = 0; i < path_size-1; ++i)
    {
        const Vertex* u = path[i+0];
        const Vertex* v = path[i+1];
        Half_Edge* edge = 0;

        for (Half_Edge* e = half_edge(space, u); ; e = next(space, e))
        {
            if (target(space, e) == v)
            {
                edge = e;
                break;
            }

            if (next(space, e) == half_edge(space, u))
            {
                break;
            }
        }

        corridormap_assert(edge != 0);
        out[i] = edge;
    }
}

namespace
{
    void extract_vertex(const Walkable_Space& space, const Half_Edge* edge, Vec2*& out_origin, float*& out_radius, Vec2*& out_obstacle_l, Vec2*& out_obstacle_r)
    {
        Vec2 p = target(space, edge)->pos;
        Vec2 l = left_side(space, edge);
        Vec2 r = right_side(space, edge);
        float radius = std::min(mag(sub(l, p)), mag(sub(r, p)));

        *out_origin++ = p;
        *out_radius++ = radius;
        *out_obstacle_l++ = l;
        *out_obstacle_r++ = r;
    }

    void extract_event(const Walkable_Space& space, const Half_Edge* edge, const Event* event, Vec2*& out_origin, float*& out_radius, Vec2*& out_obstacle_l, Vec2*& out_obstacle_r)
    {
        Vec2 p = event->pos;
        Vec2 l = left_side(space, edge, event);
        Vec2 r = right_side(space, edge, event);
        float radius = std::min(mag(sub(l, p)), mag(sub(r, p)));

        *out_origin++ = p;
        *out_radius++ = radius;
        *out_obstacle_l++ = l;
        *out_obstacle_r++ = r;
    }

    void extract_events(const Walkable_Space& space, const Half_Edge* edge, Vec2*& out_origin, float*& out_radius, Vec2*& out_obstacle_l, Vec2*& out_obstacle_r)
    {
        for (Event* evt = event(space, edge); evt != 0; evt = next(space, edge, evt))
        {
            extract_event(space, edge, evt, out_origin, out_radius, out_obstacle_l, out_obstacle_r);
        }
    }

    void set_event_curves(Vec2 prev_l, Vec2 prev_r, Vec2 curr_l, Vec2 curr_r, float epsilon, unsigned char*& out_curves)
    {
        *out_curves = curve_point << 0 | curve_point << 4;

        if (!equal(prev_l, curr_l, epsilon))
        {
            *out_curves = (*out_curves & 0xf0) | (curve_line << 0);
        }

        if (!equal(prev_r, curr_r, epsilon))
        {
            *out_curves = (*out_curves & 0x0f) | (curve_line << 4);
        }

        out_curves++;
    }

    void set_vertex_curves(Vec2 prev_l, Vec2 prev_r, Vec2 curr_l, Vec2 curr_r, float epsilon, unsigned char*& out_curves)
    {
        *out_curves = curve_point << 0 | curve_point << 4;

        if (!equal(prev_l, curr_l, epsilon))
        {
            *out_curves = (*out_curves & 0xf0) | (curve_arc_vertex << 0);
        }

        if (!equal(prev_r, curr_r, epsilon))
        {
            *out_curves = (*out_curves & 0x0f) | (curve_arc_vertex << 4);
        }

        out_curves++;
    }

    void init_event_curves(const Walkable_Space& space, const Half_Edge* edge, Vec2& prev_l, Vec2& prev_r, unsigned char*& out_curves, float epsilon)
    {
        for (Event* evt = event(space, edge); evt != 0; evt = next(space, edge, evt))
        {
            Vec2 curr_l = left_side(space, edge, evt);
            Vec2 curr_r = right_side(space, edge, evt);
            set_event_curves(prev_l, prev_r, curr_l, curr_r, epsilon, out_curves);
            prev_l = curr_l;
            prev_r = curr_r;
        }

        Vec2 curr_l = left_side(space, edge);
        Vec2 curr_r = right_side(space, edge);
        set_event_curves(prev_l, prev_r, curr_l, curr_r, epsilon, out_curves);
        prev_l = curr_l;
        prev_r = curr_r;
    }

    // non-shrunk corridor border curves could be:
    // - points = subsequent closest points are equal,
    // - lines = subsequent closest points are not equal,
    // - arcs around voronoi vertices = non-equal closest points between arriving half-edge and outgoing half-edge.
    void init_curves(const Walkable_Space& space, Half_Edge** path, int path_size, Corridor& out, float epsilon)
    {
        corridormap_assert(path_size > 0);
        unsigned char* out_curves = out.curves;

        const Half_Edge* edge = path[0];
        *out_curves++ = curve_point << 0 | curve_point << 4;

        Vec2 prev_l = right_side(space, opposite(space, edge));
        Vec2 prev_r = left_side(space, opposite(space, edge));

        init_event_curves(space, edge, prev_l, prev_r, out_curves, epsilon);

        for (int i = 1; i < path_size; ++i)
        {
            const Half_Edge* edge = path[i];
            Vec2 curr_l = right_side(space, opposite(space, edge));
            Vec2 curr_r = left_side(space, opposite(space, edge));
            set_vertex_curves(prev_l, prev_r, curr_l, curr_r, epsilon, out_curves);
            prev_l = curr_l;
            prev_r = curr_r;

            init_event_curves(space, edge, prev_l, prev_r, out_curves, epsilon);
        }
    }

    // when shrinking, some points could become arcs.
    void update_curves(Corridor& corridor, float epsilon)
    {
        corridormap_assert(corridor.num_disks > 0);

        Vec2 prev_l = corridor.border_l[0];
        Vec2 prev_r = corridor.border_r[0];

        for (int i = 1; i < corridor.num_disks; ++i)
        {
            Vec2 curr_l = corridor.border_l[i];
            Vec2 curr_r = corridor.border_r[i];

            switch (left_border_curve(corridor, i))
            {
            case curve_point:
                if (!equal(curr_l, prev_l, epsilon))
                {
                    set_left_border_curve(corridor, i, curve_arc_obstacle);
                }
                break;
            case curve_arc_obstacle:
                if (equal(curr_l, prev_l, epsilon))
                {
                    set_left_border_curve(corridor, i, curve_point);
                }
                break;
            default:
                break;
            }

            switch (right_border_curve(corridor, i))
            {
            case curve_point:
                if (!equal(curr_r, prev_r, epsilon))
                {
                    set_right_border_curve(corridor, i, curve_arc_obstacle);
                }
                break;
            case curve_arc_obstacle:
                if (equal(curr_r, prev_r, epsilon))
                {
                    set_right_border_curve(corridor, i, curve_point);
                }
                break;
            default:
                break;
            }

            prev_l = curr_l;
            prev_r = curr_r;
        }
    }
}

void extract(const Walkable_Space& space, Half_Edge** path, int path_size, Corridor& out, float epsilon)
{
    corridormap_assert(num_path_discs(space, path, path_size) <= out.max_disks);

    Vec2* out_origin = out.origin;
    float* out_radius = out.radius;
    Vec2* out_obstacle_l = out.obstacle_l;
    Vec2* out_obstacle_r = out.obstacle_r;

    for (int i = 0; i < path_size; ++i)
    {
        // swap left and right since the source vertex is extracted through opposite direction edge.
        extract_vertex(space, opposite(space, path[i]), out_origin, out_radius, out_obstacle_r, out_obstacle_l);
        extract_events(space, path[i], out_origin, out_radius, out_obstacle_l, out_obstacle_r);
        extract_vertex(space, path[i], out_origin, out_radius, out_obstacle_l, out_obstacle_r);
    }

    if (path_size > 0)
    {
        init_curves(space, path, path_size, out, epsilon);
    }

    out.num_disks = int(out_origin - out.origin);
    out.clearance = 0.f;
    out.epsilon = epsilon;
    // initialize shrunk borders to the obstacle closest points.
    memcpy(out.border_l, out.obstacle_l, out.num_disks*sizeof(Vec2));
    memcpy(out.border_r, out.obstacle_r, out.num_disks*sizeof(Vec2));
}

void shrink(Corridor& corridor, float clearance)
{
    corridor.clearance = clearance;

    for (int i = 0; i < corridor.num_disks; ++i)
    {
        Vec2 origin = corridor.origin[i];
        float radius = corridor.radius[i];
        corridormap_assert(radius > clearance);
        Vec2 l = corridor.obstacle_l[i];
        Vec2 r = corridor.obstacle_r[i];
        Vec2 l_b = add(l, scale(normalized(sub(origin, l)), clearance));
        Vec2 r_b = add(r, scale(normalized(sub(origin, r)), clearance));
        corridor.border_l[i] = l_b;
        corridor.border_r[i] = r_b;
    }

    if (corridor.num_disks > 0)
    {
        update_curves(corridor, corridor.epsilon);
    }
}

namespace
{
    float orient(Vec2 o, Vec2 a, Vec2 b)
    {
        return det(sub(a, o), sub(b, o));
    }

    void add_portal(Corridor& corridor, Vec2 l, Vec2 r)
    {
        corridor.portal_l[corridor.num_portals] = l;
        corridor.portal_r[corridor.num_portals] = r;
        corridor.num_portals++;
    }

    bool tess_arc(Corridor& corridor, Vec2 portal_side, Vec2 origin, Vec2 from, Vec2 to, float radius, float max_step, bool ccw)
    {
        Vec2 da = normalized(sub(from, origin));
        Vec2 db = normalized(sub(to, origin));
        float arc_angle = acosf(dot(da, db));

        if (orient(origin, from, to) > 0.f != ccw)
        {
            arc_angle = 2.f*CORRIDORMAP_PI - arc_angle;
        }

        float arc_len = arc_angle*radius;
        int steps = int(floorf(arc_len / max_step));
        float theta = (ccw ? +arc_angle : -arc_angle) / float(steps);
        float start_angle = atan2(da.y, da.x);

        if (corridor.num_portals + steps >= corridor.max_portals)
        {
            return false;
        }

        for (int i = 0; i < steps-1; ++i)
        {
            Vec2 p = make_vec2(radius*cosf(start_angle + (i+1)*theta), radius*sinf(start_angle + (i+1)*theta));
            p = add(origin, p);
            add_portal(corridor, ccw ? p : portal_side, ccw ? portal_side : p);
        }

        add_portal(corridor, ccw ? to : portal_side, ccw ? portal_side : to);
        return true;
    }
}

int triangulate(Corridor& corridor, float arc_step_len)
{
    float clearance = corridor.clearance;
    corridor.num_portals = 0;

    if (corridor.num_disks > 0 && corridor.max_portals > 0)
    {
        add_portal(corridor, corridor.border_l[0], corridor.border_r[0]);
    }

    for (int disk = 0; disk < corridor.num_disks-1; ++disk)
    {
        Vec2 l0 = corridor.border_l[disk+0];
        Vec2 l1 = corridor.border_l[disk+1];
        Curve curve_l = left_border_curve(corridor, disk+1);

        Vec2 r0 = corridor.border_r[disk+0];
        Vec2 r1 = corridor.border_r[disk+1];
        Curve curve_r = right_border_curve(corridor, disk+1);

        if (corridor.num_portals + 2 >= corridor.max_portals)
        {
            return disk;
        }

        if (curve_r != curve_point)
        {
            if (curve_r == curve_arc_obstacle)
            {
                if (!tess_arc(corridor, l0, corridor.obstacle_r[disk], r0, r1, clearance, arc_step_len, false))
                {
                    return disk;
                }
            }
            else
            {
                add_portal(corridor, l0, r1);
            }
        }

        if (curve_l != curve_point)
        {
            if (curve_l == curve_arc_obstacle)
            {
                if (!tess_arc(corridor, r1, corridor.obstacle_l[disk], l0, l1, clearance, arc_step_len, true))
                {
                    return disk;
                }
            }
            else
            {
                add_portal(corridor, l1, r1);
            }
        }
    }

    return corridor.num_disks;
}

int find_closest_disk(const Corridor& corridor, Vec2 point)
{
    float min_dist = FLT_MAX;
    int result = 0;

    for (int i = 0; i < corridor.num_disks; ++i)
    {
        float dist_sq = mag_sq(sub(point, corridor.origin[i]));

        if (dist_sq < min_dist)
        {
            min_dist = dist_sq;
            result = i;
        }
    }

    return result;
}

// reference: "Simple Stupid Funnel Algorithm",
// http://digestingduck.blogspot.co.at/2010/03/simple-stupid-funnel-algorithm.html
int find_shortest_path(const Corridor& corridor, Vec2 source, Vec2 target, int first_portal, int last_portal, Vec2* path, int max_path_size)
{
    corridormap_assert(corridor.num_disks > 0);
    corridormap_assert(corridor.num_portals > 0);
    corridormap_assert(first_portal >= 0 && first_portal < corridor.num_portals);
    corridormap_assert(last_portal >= 0 && last_portal < corridor.num_portals);
    corridormap_assert(first_portal <= last_portal);
    corridormap_assert(max_path_size > 0);

    int path_size = 0;
    int left_idx = 0;
    int right_idx = 0;
    Vec2 apex = source;
    Vec2 left = source;
    Vec2 right = source;

    path[path_size++] = apex;

    for (int i = first_portal; i <= last_portal+1 && path_size < max_path_size; ++i)
    {
        Vec2 portal_l = target;
        Vec2 portal_r = target;

        if (i < last_portal)
        {
            portal_l = corridor.portal_l[i];
            portal_r = corridor.portal_r[i];
        }

        if (orient(apex, portal_l, left) >= 0.f)
        {
            if (equal(apex, left, 1e-6f) || orient(apex, right, portal_l) > 0.f)
            {
                left = portal_l;
                left_idx = i;
            }
            else
            {
                path[path_size++] = right;
                apex = right;
                left = apex;
                left_idx = right_idx;
                i = right_idx;
                continue;
            }
        }

        if (orient(apex, right, portal_r) >= 0.f)
        {
            if (equal(apex, right, 1e-6f) || orient(apex, portal_r, left) > 0.f)
            {
                right = portal_r;
                right_idx = i;
            }
            else
            {
                path[path_size++] = left;
                apex = left;
                right = apex;
                right_idx = left_idx;
                i = right_idx;
                continue;
            }
        }
    }

    if (path_size < max_path_size)
    {
        path[path_size++] = target;
    }

    return path_size;
}

}
