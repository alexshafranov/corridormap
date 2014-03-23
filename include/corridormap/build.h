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

#ifndef CORRIDORMAP_BUILD_H_
#define CORRIDORMAP_BUILD_H_

#include "corridormap/types.h"
#include "corridormap/memory.h"

namespace corridormap {

static const float CORRIDORMAP_SQRT_2   = 1.41421356f;
static const float CORRIDORMAP_PI       = 3.14159265f;

// maximum distance for points and lines.
// computed such that distance mesh "covers" the full render target in ortho projection.
inline float max_distance(float scene_bbox_min[2], float scene_bbox_max[2])
{
    float w = scene_bbox_max[0] - scene_bbox_min[0];
    float h = scene_bbox_max[1] - scene_bbox_min[1];
    float s = (w > h) ? w : h;
    return s * CORRIDORMAP_SQRT_2;
}

// build polygon distance mesh, suitable to be passed to render interface. polygon must be convex.
triangle_list build_distance_mesh(polygon obstacle, float max_dist, float max_error, memory* output);

}

#endif
