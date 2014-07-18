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

#include "corridormap/assert.h"
#include "corridormap/vec2.h"
#include "corridormap/runtime.h"


namespace corridormap {

// reference: "Simple Stupid Funnel Algorithm", [http://digestingduck.blogspot.co.at/2010/03/simple-stupid-funnel-algorithm.html]
int find_shortest_path(const Corridor& corridor, Vec2 source, Vec2 target, int first_portal, int last_portal, Vec2* path, int max_path_size)
{
    corridormap_assert(corridor.num_disks > 0);
    corridormap_assert(corridor.num_portals > 0);
    corridormap_assert(first_portal >= 0 && first_portal < corridor.num_portals);
    corridormap_assert(last_portal >= 0 && last_portal < corridor.num_portals);
    corridormap_assert(first_portal <= last_portal);
    corridormap_assert(max_path_size > 0);

    int path_size = 0;
    int left_idx = 0;
    int right_idx = 0;
    Vec2 apex = source;
    Vec2 left = source;
    Vec2 right = source;

    path[path_size++] = apex;

    for (int i = first_portal; i <= last_portal+1 && path_size < max_path_size; ++i)
    {
        Vec2 portal_l = target;
        Vec2 portal_r = target;

        if (i < last_portal)
        {
            portal_l = corridor.portal_l[i];
            portal_r = corridor.portal_r[i];
        }

        if (orient(apex, portal_l, left) >= 0.f)
        {
            if (equal(apex, left, 1e-6f) || orient(apex, right, portal_l) > 0.f)
            {
                left = portal_l;
                left_idx = i;
            }
            else
            {
                path[path_size++] = right;
                apex = right;
                left = apex;
                left_idx = right_idx;
                i = right_idx;
                continue;
            }
        }

        if (orient(apex, right, portal_r) >= 0.f)
        {
            if (equal(apex, right, 1e-6f) || orient(apex, portal_r, left) > 0.f)
            {
                right = portal_r;
                right_idx = i;
            }
            else
            {
                path[path_size++] = left;
                apex = left;
                right = apex;
                right_idx = left_idx;
                i = right_idx;
                continue;
            }
        }
    }

    if (path_size < max_path_size)
    {
        path[path_size++] = target;
    }

    return path_size;
}

}
