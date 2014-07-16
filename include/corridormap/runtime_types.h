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

enum { null_idx = ~0u };

struct Vec2
{
    float x;
    float y;
};

// Vertex of the medial axis graph.
struct Vertex
{
    // pool link.
    int link;
    // index of the first outgoing half-edge.
    int half_edge;
    // position.
    Vec2 pos;
};

// Edge direction of medial axis graph.
struct Half_Edge
{
    // link to the next incident half-edge in CCW order, forms cyclic list of edges around each vertex.
    int next;
    // index of the target vertex.
    int target;
    // index of the head of the event list when going this half-edge direction.
    int event;
    // closest points on left and right obstacles along the half-edge direction.
    Vec2 sides[2];
};

// Medial axis graph edge.
struct Edge
{
    // pool link.
    int link;
    // two halves of the edge.
    Half_Edge dir[2];
};

// Edge event point: position on the edge where left or right closest obstacle changes.
struct Event
{
    // pool link.
    int link;
    // next event for both half-edge directions.
    int next[2];
    // position.
    Vec2 pos;
    // closest points on left and right obstacles.
    Vec2 sides[2];
};

// Array-based free list.
template <typename T>
struct Pool
{
    // head of the list of allocated objects.
    int head;
    // tail of the list of allocated objects.
    int tail;
    // head of the list of free objects.
    int head_free;
    // total number of allocated objects.
    int num_items;
    // size of the items array.
    int max_items;
    // array of items of size max_items.
    T* items;
};

// Medial axis graph with edges and vertices annotated with closest obstacle information.
struct Walkable_Space
{
    // pool of vertices.
    Pool<Vertex> vertices;
    // pool of edges.
    Pool<Edge> edges;
    // pool of edge events.
    Pool<Event> events;
};

// Type of the curve connecting subsequent corridor border points.
enum Border_Curve_Type
{
    // next point is equal to the current one.
    border_type_point = 0,
    // line between current point and the next one.
    border_type_line,
    // circle arc around edge vertex.
    border_type_arc_vertex,
    // circle arc around obstacle vertex.
    border_type_arc_obstacle,
};

// Set of disks describing a path corridor.
struct Corridor
{
    // num allocated disks.
    int max_disks;
    // num disks in a corridor.
    int num_disks;
    // current clearance value (>= 0 for shrunk corridors).
    float clearance;
    // epsilon used to compare border points.
    float epsilon;
    // [0..num_disks).
    Vec2* origin;
    // [0..num_disks).
    float* radius;
    // left side closest obstacle point. [0..num_disks).
    Vec2* obstacle_l;
    // right side closest obstacle point. [0..num_disks).
    Vec2* obstacle_r;
    // left side shrunk corridor border point. [0..num_disks).
    Vec2* border_l;
    // right side shrunk corridor border point. [0..num_disks).
    Vec2* border_r;
    // packed curve type for left and right borders.
    unsigned char* curves;
};

}

#endif
