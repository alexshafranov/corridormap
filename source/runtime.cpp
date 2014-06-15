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

#include <string.h>
#include "corridormap/assert.h"
#include "corridormap/memory.h"
#include "corridormap/vec2.h"
#include "corridormap/runtime.h"

namespace corridormap {

voronoi_diagram create_voronoi_diagram(memory* mem, int max_vertices, int max_edges, int max_events)
{
    voronoi_diagram result;
    memset(&result, 0, sizeof(result));

    result.vertices.elems = allocate<vertex>(mem, max_vertices);
    result.edges.elems = allocate<edge>(mem, max_edges);
    result.events.elems = allocate<event>(mem, max_events);

    result.vertices.max_elems = max_vertices;
    result.edges.max_elems = max_edges;
    result.events.max_elems = max_events;

    init(result.vertices);
    init(result.edges);
    init(result.events);

    return result;
}

void destroy(memory* mem, voronoi_diagram& d)
{
    mem->deallocate(d.vertices.elems);
    mem->deallocate(d.edges.elems);
    mem->deallocate(d.events.elems);
    memset(&d, 0, sizeof(d));
}

namespace
{
    bool is_ccw(vec2 u, vec2 v1, vec2 v2)
    {
        vec2 d1 = sub(v1, u);
        vec2 d2 = sub(v2, u);
        return d1.x*d2.y - d2.x*d1.y > 0.f;
    }

    half_edge* get_half_edge(edge* edges, int idx)
    {
        return edges[idx>>1].dir + (idx&1);
    }

    void add_half_edge(vertex* vertices, edge* edges, int vert, int h_edge)
    {
        int head = vertices[vert].half_edge;
        half_edge* new_half_edge = get_half_edge(edges, h_edge);

        if (head == null_idx)
        {
            vertices[vert].half_edge = h_edge;
            new_half_edge->next = h_edge;
        }
        else
        {
            vec2 u = vertices[vert].pos;
            vec2 v2 = vertices[new_half_edge->target].pos;

            int insert_after = null_idx;
            int curr = head;

            for (;;)
            {
                half_edge* curr_half_edge = get_half_edge(edges, curr);

                vec2 v1 = vertices[curr_half_edge->target].pos;

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
                half_edge* tail_half_edge = get_half_edge(edges, head);

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
                half_edge* insert_after_half_edge = get_half_edge(edges, insert_after);
                int next = insert_after_half_edge->next;
                insert_after_half_edge->next = h_edge;
                new_half_edge->next = next;
            }
        }
    }

    void append_event(edge* edges, event* events, int h_edge, int evt)
    {
        int dir = h_edge & 1;
        half_edge* h = get_half_edge(edges, h_edge);
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

    void prepend_event(edge* edges, event* events, int h_edge, int evt)
    {
        int dir = h_edge & 1;
        half_edge* h = get_half_edge(edges, h_edge);
        int head = h->event;
        events[evt].next[dir] = head;
        h->event = evt;
    }
}

vertex* create_vertex(voronoi_diagram& diagram, vec2 pos)
{
    vertex* new_vertex = allocate(diagram.vertices);
    new_vertex->half_edge = null_idx;
    new_vertex->pos = pos;
    return new_vertex;
}

edge* create_edge(voronoi_diagram& diagram, int u, int v)
{
    edge* new_edge = allocate(diagram.edges);
    new_edge->dir[0].target = v;
    new_edge->dir[1].target = u;
    new_edge->dir[0].event = null_idx;
    new_edge->dir[1].event = null_idx;
    int new_edge_idx = int(new_edge - diagram.edges.elems);
    add_half_edge(diagram.vertices.elems, diagram.edges.elems, u, new_edge_idx*2 + 0);
    add_half_edge(diagram.vertices.elems, diagram.edges.elems, v, new_edge_idx*2 + 1);
    return new_edge;
}

event* create_event(voronoi_diagram& diagram, vec2 pos, int edge)
{
    event* new_event = allocate(diagram.events);
    new_event->pos = pos;
    new_event->next[0] = null_idx;
    new_event->next[1] = null_idx;
    int new_event_idx = int(new_event - diagram.events.elems);
    append_event(diagram.edges.elems, diagram.events.elems, edge*2 + 0, new_event_idx);
    prepend_event(diagram.edges.elems, diagram.events.elems, edge*2 + 1, new_event_idx);
    return new_event;
}

}
