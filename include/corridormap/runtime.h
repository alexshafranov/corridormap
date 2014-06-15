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

// allocate and initialize voronoi diagram.
Voronoi_Diagram create_voronoi_diagram(Memory* mem, int max_vertices, int max_edges, int max_events);
// destroy voronoi diagram.
void destroy(Memory* mem, Voronoi_Diagram& d);

// creates a new vertex with the specified position.
Vertex* create_vertex(Voronoi_Diagram& diagram, Vec2 pos);

// creates an edge between vertices u and v.
Edge* create_edge(Voronoi_Diagram& diagram, int u, int v);

// creates a new event and appends it to the specified edge.
Event* create_event(Voronoi_Diagram& diagram, Vec2 pos, int edge);

}

namespace corridormap {

template <typename T>
void init(Free_List<T>& lst)
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
T* allocate(Free_List<T>& lst)
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

template <typename T>
inline T* ptr(const Free_List<T>& lst, int offset)
{
    return (offset != null_idx) ? lst.items + offset : 0;
}

template <typename T>
inline T* first(const Free_List<T>& lst)
{
    return ptr(lst, lst.head);
}

template <typename T>
inline T* next(const Free_List<T>& lst, T* item)
{
    int next_index = lst.items[int(item - lst.items)].link;
    return ptr(lst, next_index);
}

inline Edge* edge(const Voronoi_Diagram& diagram, Half_Edge* e)
{
    const char* a = reinterpret_cast<const char*>(diagram.edges.items);
    const char* b = reinterpret_cast<const char*>(e);
    int diff = int(b - a);
    return diagram.edges.items + diff/sizeof(Edge);
}

inline Half_Edge* opposite(const Voronoi_Diagram& diagram, Half_Edge* e)
{
    Edge* p = edge(diagram, e);
    int dir = int(e - p->dir);
    return p->dir + (dir^1);
}

inline Vertex* target(const Voronoi_Diagram& diagram, Half_Edge* e)
{
    return ptr(diagram.vertices, e->target);
}

inline Vertex* source(const Voronoi_Diagram& diagram, Half_Edge* e)
{
    return target(diagram, opposite(diagram, e));
}

inline Event* event(const Voronoi_Diagram& diagram, Half_Edge* e)
{
    return ptr(diagram.events, e->event);
}

inline Event* next(const Voronoi_Diagram& diagram, Event* e, int direction)
{
    return ptr(diagram.events, e->next[direction]);
}

}

#endif
