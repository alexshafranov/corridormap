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
#include "corridormap/assert.h"
#include "corridormap/memory.h"
#include "corridormap/vec2.h"
#include "corridormap/runtime.h"

namespace corridormap {

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
        return d1.x*d2.y - d2.x*d1.y > 0.f;
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

int num_path_discs(const Walkable_Space& space, const Half_Edge** path, int path_size)
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
    }

    result += path_size + 1;

    return result;
}

Corridor create_corridor(Memory* mem, int max_discs)
{
    Corridor result;
    memset(&result, 0, sizeof(result));
    result.origins = allocate<Vec2>(mem, max_discs);
    result.radii = allocate<float>(mem, max_discs);
    result.left = allocate<Vec2>(mem, max_discs);
    result.right = allocate<Vec2>(mem, max_discs);
    result.max_discs = max_discs;
    return result;
}

void destroy(Memory* mem, Corridor& c)
{
    mem->deallocate(c.origins);
    mem->deallocate(c.radii);
    mem->deallocate(c.left);
    mem->deallocate(c.right);
    memset(&c, 0, sizeof(c));
}

namespace
{
    void extract_vertex(const Walkable_Space& space, const Half_Edge* edge, Vec2* out_origins, float* out_radii, Vec2* out_left, Vec2* out_right)
    {
        Vec2 p = target(space, edge)->pos;
        Vec2 l = left_side(space, edge);
        Vec2 r = right_side(space, edge);
        float radius = std::min(len(sub(l, p)), len(sub(r, p)));

        *out_origins++ = p;
        *out_radii++ = radius;
        *out_left++ = l;
        *out_right++ = r;
    }

    void extract_event(const Walkable_Space& space, const Half_Edge* edge, const Event* event, Vec2* out_origins, float* out_radii, Vec2* out_left, Vec2* out_right)
    {
        Vec2 p = event->pos;
        Vec2 l = left_side(space, edge, event);
        Vec2 r = right_side(space, edge, event);
        float radius = std::min(len(sub(l, p)), len(sub(r, p)));

        *out_origins++ = p;
        *out_radii++ = radius;
        *out_left++ = l;
        *out_right++ = r;
    }
}

void extract(const Walkable_Space& space, const Half_Edge** path, int path_size, Corridor& out)
{
    corridormap_assert(path_size > 0);
    corridormap_assert(num_path_discs(space, path, path_size) <= out.max_discs);

    Vec2* out_origins = out.origins;
    float* out_radii = out.radii;
    Vec2* out_left = out.left;
    Vec2* out_right = out.right;

    const Half_Edge* edge = opposite(space, path[0]);
    extract_vertex(space, edge, out_origins, out_radii, out_left, out_right);

    for (int i = 0; i < path_size; ++i)
    {
        const Half_Edge* edge = path[i];

        for (Event* evt = event(space, edge); evt != 0; evt = next(space, edge, evt))
        {
            extract_event(space, edge, evt, out_origins, out_radii, out_left, out_right);
        }

        extract_vertex(space, edge, out_origins, out_radii, out_left, out_right);
    }

    out.num_discs = int(out_origins - out.origins);
}

}
