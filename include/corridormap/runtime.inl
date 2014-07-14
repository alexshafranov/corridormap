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

#ifndef CORRIDORMAP_RUNTIME_INL_
#define CORRIDORMAP_RUNTIME_INL_

namespace corridormap {

/// vertex

inline Half_Edge* half_edge(const Walkable_Space& space, const Vertex* v)
{
    Edge* edge = ptr(space.edges, v->half_edge >> 1);
    return edge ? edge->dir + (v->half_edge & 1) : 0;
}

/// half-edge

inline Edge* edge(const Walkable_Space& space, const Half_Edge* e)
{
    const char* a = reinterpret_cast<const char*>(space.edges.items);
    const char* b = reinterpret_cast<const char*>(e);
    int diff = int(b - a);
    return space.edges.items + diff/sizeof(Edge);
}

inline Half_Edge* opposite(const Walkable_Space& space, const Half_Edge* e)
{
    Edge* p = edge(space, e);
    int dir = int(e - p->dir);
    return p->dir + (dir^1);
}

inline Vertex* target(const Walkable_Space& space, const Half_Edge* e)
{
    return ptr(space.vertices, e->target);
}

inline Vertex* source(const Walkable_Space& space, const Half_Edge* e)
{
    return target(space, opposite(space, e));
}

inline Half_Edge* next(const Walkable_Space& space, const Half_Edge* e)
{
    Edge* edge = ptr(space.edges, e->next >> 1);
    return edge ? edge->dir + (e->next & 1) : 0;
}

inline Event* event(const Walkable_Space& space, const Half_Edge* e)
{
    return ptr(space.events, e->event);
}

inline Event* next(const Walkable_Space& space, const Half_Edge* half_edge, const Event* e)
{
    Edge* p = edge(space, half_edge);
    int dir = int(half_edge - p->dir);
    return ptr(space.events, e->next[dir]);
}

inline Vec2 left_side(const Walkable_Space& space, const Half_Edge* half_edge, const Event* e)
{
    Edge* p = edge(space, half_edge);
    int dir = int(half_edge - p->dir);
    return e->sides[dir^0];
}

inline Vec2 right_side(const Walkable_Space& space, const Half_Edge* half_edge, const Event* e)
{
    Edge* p = edge(space, half_edge);
    int dir = int(half_edge - p->dir);
    return e->sides[dir^1];
}

inline Vec2 left_side(const Walkable_Space&, const Half_Edge* half_edge)
{
    return half_edge->sides[0];
}

inline Vec2 right_side(const Walkable_Space&, const Half_Edge* half_edge)
{
    return half_edge->sides[1];
}

/// edge

inline Vertex* target(const Walkable_Space& space, const Edge* e)
{
    return space.vertices.items + e->dir[0].target;
}

inline Vertex* source(const Walkable_Space& space, const Edge* e)
{
    return space.vertices.items + e->dir[1].target;
}

}

#endif
