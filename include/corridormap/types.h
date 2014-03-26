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

#ifndef CORRIDORMAP_TYPES_H_
#define CORRIDORMAP_TYPES_H_

namespace corridormap {

// 2d ostacle polygon
struct polygon
{
    // number of vertices.
    unsigned nverts;
    // vertcies in ccw order. length is nverts*2.
    float* vertices;
};

// 3d triangle list suitable for rendering.
struct triangle_list
{
    // number of triangles.
    unsigned ntris;
    // vertex position data. length is ntris*3*3.
    float* vertices;
};

// 2d bounding box.
struct bbox2
{
    float min[2];
    float max[2];
};

// obstacles represented as a set of 2d convex polygons. polys are stored in CCW order.
struct footprint
{
    // the number of polygons.
    int num_polys;
    // the total number of vertices.
    int num_verts;
    // x coords indexed in [0..total_verts] range
    float* x;
    // y coords indexed in [0..total_verts] range
    float* y;
    // array of vertex counts per poly, indexed in [0..num_polys]
    int* num_poly_verts;
};

}

#endif
