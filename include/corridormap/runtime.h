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
#include "corridormap/pool.h"

namespace corridormap { class Memory; }

namespace corridormap {

// allocate and initialize walkable space data.
Walkable_Space create_walkable_space(Memory* mem, int max_vertices, int max_edges, int max_events);
// destroy walkable space.
void destroy(Memory* mem, Walkable_Space& d);

// creates a new vertex with the specified position.
Vertex* create_vertex(Walkable_Space& space, Vec2 pos);
// creates an edge between vertices u and v.
Edge* create_edge(Walkable_Space& space, int u, int v);
// creates a new event and appends it to the specified edge.
Event* create_event(Walkable_Space& space, Vec2 pos, int edge);

/// Vertex

// first half-edge which has this vertex as a source.
Half_Edge* half_edge(const Walkable_Space& space, const Vertex* v);
// number of edges incident to this vertex.
int degree(const Walkable_Space& space, const Vertex* vertex);

/// Half_Edge

// the edge that owns this half-edge.
Edge* edge(const Walkable_Space& space, const Half_Edge* e);
// the opposite direction half-edge.
Half_Edge* opposite(const Walkable_Space& space, const Half_Edge* e);
// target vertex of this half-edge.
Vertex* target(const Walkable_Space& space, const Half_Edge* e);
// source vertex of this half-edge.
Vertex* source(const Walkable_Space& space, const Half_Edge* e);
// next half-edge in CCW order around source vertex of this half-edge.
Half_Edge* next(const Walkable_Space& space, const Half_Edge* e);
// first event in the direction of this half-edge.
Event* event(const Walkable_Space& space, const Half_Edge* e);
// next event in the direction of this half-edge.
Event* next(const Walkable_Space& space, const Half_Edge* half_edge, const Event* e);
// left side at the specified event point along this half-edge direction.
Vec2 left_side(const Walkable_Space& space, const Half_Edge* half_edge, const Event* e);
// right side at the specified event point along this half-edge direction.
Vec2 right_side(const Walkable_Space& space, const Half_Edge* half_edge, const Event* e);
// left side at the target vertex along this half-edge direction.
Vec2 left_side(const Walkable_Space& space, const Half_Edge* half_edge);
// right side at the target vertex along this half-edge direction.
Vec2 right_side(const Walkable_Space& space, const Half_Edge* half_edge);

/// Edge

// target vertex in the default direction of this edge (i.e. first half-edge).
Vertex* target(const Walkable_Space& space, const Edge* e);
// source vertex in the default direction of this edge (i.e. first half-edge).
Vertex* source(const Walkable_Space& space, const Edge* e);

/// Corridor

// convert vertex path to half-edge path.
void vertex_to_edge_path(const Walkable_Space& space, Vertex** path, int path_size, Half_Edge** out);
// returns number of event points and vertices along the half-edge path.
int num_path_discs(const Walkable_Space& space, Half_Edge** path, int path_size);

// allocate corridor.
Corridor create_corridor(Memory* mem, int max_disks, int max_portals);
// destroy corridor.
void destroy(Memory* mem, Corridor& corridor);

// extract corridor from half-edge path. epsilon is used to test equality of border points.
void extract(const Walkable_Space& space, Half_Edge** path, int path_size, Corridor& out, float epsilon=0.2f);
// shrink corridor to the new clearance value.
void shrink(Corridor& corridor, float clearance);
// triangulate corridor (stores portal edges).
// returns number of disks processed. result < num_disks if there's not enough space allocated for portals.
int triangulate(Corridor& corridor, float arc_step_len);

// linear search for the disk closest to the specified point.
int find_closest_disk(const Corridor& corridor, Vec2 point);
// finds shortest path inside the corridor, by applying funnel algorithm. returns resulting path size.
int find_shortest_path(const Corridor& corridor, Vec2 source, Vec2 target, int first_portal, int last_portal, Vec2* path, int max_path_size);

// unpack curve type for the connection between disk_index-1 and disk_index.
Curve left_border_curve(const Corridor& corridor, int disk_index);
Curve right_border_curve(const Corridor& corridor, int disk_index);

// set border curve type.
void set_left_border_curve(const Corridor& corridor, int disk_index, Curve type);
void set_right_border_curve(const Corridor& corridor, int disk_index, Curve type);

}

#include "corridormap/runtime.inl"

#endif
