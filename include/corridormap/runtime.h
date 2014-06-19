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

#ifndef CORRIDORMAP_RUNTIME_H_
#define CORRIDORMAP_RUNTIME_H_

#include "corridormap/runtime_types.h"

namespace corridormap { class Memory; }

namespace corridormap {

// allocate and initialize walkable space data.
Walkable_Space create_walkable_space(Memory* mem, int max_vertices, int max_edges, int max_events);
// destroy voronoi diagram.
void destroy(Memory* mem, Walkable_Space& d);

// creates a new vertex with the specified position.
Vertex* create_vertex(Walkable_Space& diagram, Vec2 pos);

// creates an edge between vertices u and v.
Edge* create_edge(Walkable_Space& diagram, int u, int v);

// creates a new event and appends it to the specified edge.
Event* create_event(Walkable_Space& diagram, Vec2 pos, int edge);

}

namespace corridormap {

template <typename T>
void init(Pool<T>& lst)
{
    lst.head = null_idx;
    lst.tail = null_idx;
    lst.head_free = 0;
    lst.num_items = 0;

    for (int i = 0; i < lst.max_items - 1; ++i)
    {
        lst.items[i].link = i + 1;
    }

    lst.items[lst.max_items - 1].link = null_idx;
}

template <typename T>
T* allocate(Pool<T>& lst)
{
    if (lst.head_free == null_idx)
    {
        return 0;
    }

    int result_index = lst.head_free;
    T* result = lst.items + result_index;

    lst.head_free = lst.items[lst.head_free].link;

    if (lst.head == null_idx)
    {
        lst.head = result_index;
        lst.tail = result_index;
        lst.items[result_index].link = null_idx;
    }
    else
    {
        lst.items[lst.tail].link = result_index;
        lst.tail = result_index;
        lst.items[result_index].link = null_idx;
    }

    lst.num_items++;

    return result;
}

/// Pool

template <typename T>
inline T* ptr(const Pool<T>& lst, int offset)
{
    return (offset != null_idx) ? lst.items + offset : 0;
}

template <typename T>
inline T* first(const Pool<T>& lst)
{
    return ptr(lst, lst.head);
}

template <typename T>
inline T* next(const Pool<T>& lst, T* item)
{
    int next_index = lst.items[int(item - lst.items)].link;
    return ptr(lst, next_index);
}

/// Vertex

inline Half_Edge* half_edge(const Walkable_Space& space, Vertex* v)
{
    Edge* edge = ptr(space.edges, v->half_edge >> 1);
    return edge ? edge->dir + (v->half_edge & 1) : 0;
}

/// Half_Edge

inline Edge* edge(const Walkable_Space& space, Half_Edge* e)
{
    const char* a = reinterpret_cast<const char*>(space.edges.items);
    const char* b = reinterpret_cast<const char*>(e);
    int diff = int(b - a);
    return space.edges.items + diff/sizeof(Edge);
}

inline Half_Edge* opposite(const Walkable_Space& space, Half_Edge* e)
{
    Edge* p = edge(space, e);
    int dir = int(e - p->dir);
    return p->dir + (dir^1);
}

inline Vertex* target(const Walkable_Space& space, Half_Edge* e)
{
    return ptr(space.vertices, e->target);
}

inline Vertex* source(const Walkable_Space& space, Half_Edge* e)
{
    return target(space, opposite(space, e));
}

inline Half_Edge* next(const Walkable_Space& space, Half_Edge* e)
{
    Edge* edge = ptr(space.edges, e->next >> 1);
    return edge ? edge->dir + (e->next & 1) : 0;
}

inline Event* event(const Walkable_Space& space, Half_Edge* e)
{
    return ptr(space.events, e->event);
}

/// Event

inline Event* next(const Walkable_Space& space, Event* e, int direction)
{
    return ptr(space.events, e->next[direction]);
}

}

#endif
