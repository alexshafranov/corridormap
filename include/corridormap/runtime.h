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

namespace corridormap { class memory; }

namespace corridormap {

// allocate and initialize voronoi diagram.
voronoi_diagram create_voronoi_diagram(memory* mem, int max_vertices, int max_edges, int max_events);
// destroy voronoi diagram.
void destroy(memory* mem, voronoi_diagram& d);

// creates a new vertex with the specified position.
vertex* create_vertex(voronoi_diagram& diagram, vec2 pos);

// creates an edge between vertices u and v.
edge* create_edge(voronoi_diagram& diagram, int u, int v);

// creates a new event and appends it to the specified edge.
event* create_event(voronoi_diagram& diagram, vec2 pos, int edge);

}

namespace corridormap {

template <typename T>
void init(free_list<T>& lst)
{
    lst.head = null_idx;
    lst.num_elems = 0;
    lst.first_free = 0;

    for (int i = 0; i < lst.max_elems - 1; ++i)
    {
        lst.elems[i].link = i + 1;
    }

    lst.elems[lst.max_elems - 1].link = null_idx;
}

template <typename T>
T* allocate(free_list<T>& lst)
{
    if (lst.first_free == null_idx)
    {
        return 0;
    }

    int result_index = lst.first_free;
    T* result = lst.elems + result_index;

    lst.first_free = lst.elems[lst.first_free].link;

    if (lst.head == null_idx)
    {
        lst.head = result_index;
    }
    else
    {
        lst.elems[result_index].link = lst.elems[lst.head].link;
        lst.head = result_index;
    }

    lst.num_elems++;

    return result;
}

}

#endif
