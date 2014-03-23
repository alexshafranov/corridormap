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

#include <math.h>
#include "corridormap/assert.h"
#include "corridormap/build.h"

namespace corridormap {

triangle_list build_distance_mesh(polygon obstacle, float max_dist, float max_error, memory* output)
{
    corridormap_assert(max_dist > max_error);

    triangle_list result;
    result.ntris = 0;
    result.vertices = 0;

    const float cone_half_angle = acos((max_dist - max_error)/max_dist);
    const float cone_angle = 2.f * cone_half_angle;
    const unsigned cone_triangle_count = static_cast<unsigned>(floor(CORRIDORMAP_PI / cone_half_angle));

    const size_t vertex_size = 3 * sizeof(float);
    const size_t triangle_size = 3 * vertex_size;

    const unsigned ntris = obstacle.nverts * cone_triangle_count;

    float* vertices = allocate<float>(output, ntris*triangle_size);

    if (!vertices)
    {
        return result;
    }

    for (unsigned i = 0; i < obstacle.nverts; ++i)
    {
        float p_x = obstacle.vertices[i*2+0];
        float p_y = obstacle.vertices[i*2+1];

        float* cone = vertices + i*cone_triangle_count*triangle_size;

        for (unsigned j = 0; j < cone_triangle_count; ++j)
        {
            float* a = cone + j*triangle_size + 0;
            float* b = cone + j*triangle_size + 1;
            float* c = cone + j*triangle_size + 2;

            a[0] = p_x;
            a[1] = p_y;
            a[2] = 0.f;

            b[0] = max_dist * cos((j+0)*cone_angle);
            b[1] = max_dist * sin((j+0)*cone_angle);
            b[2] = max_dist;

            c[0] = max_dist * cos((j+1)*cone_angle);
            c[1] = max_dist * sin((j+1)*cone_angle);
            c[2] = max_dist;
        }
    }

    result.vertices = vertices;
    result.ntris = ntris;

    return result;
}

}
