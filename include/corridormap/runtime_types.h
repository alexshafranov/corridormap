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

#ifndef CORRIDORMAP_RUNTIME_TYPES_H_
#define CORRIDORMAP_RUNTIME_TYPES_H_

namespace corridormap {

struct vec2
{
    float x;
    float y;
};

enum { max_vertex_sides = 4 };

struct vertex
{
    int next;
    int half_edge;
    vec2 pos;
    vec2 sides[max_vertex_sides];
};

struct half_edge
{
    int next;
    int source;
    int event;
};

struct event
{
    int next[2];
    vec2 position;
    vec2 sides[2];
};

template <typename T>
struct free_list
{
    int head;
    int first_free;
    int num_elems;
    int max_elems;
    T* elems;
};

struct voronoi_diagram
{
    free_list<vertex> vertices;
    free_list<half_edge> half_edges;
    free_list<event> events;
};

}

#endif
