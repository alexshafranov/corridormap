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

const sampler_t sampler = CLK_NORMALIZED_COORD_FALSE|CLK_ADDRESS_CLAMP|CLK_FILTER_NEAREST;

kernel mark_poi(
    read_only  image2d_t voronoi,
    write_only image2d_t vertex_marks,
    write_only image2d_t edge_marks,
    const int width,
    const int height)
{
    int gid0 = get_global_id(0);
    int gid1 = get_global_id(1);

    if (gid0 >= width || gid1 >= height)
    {
        return;
    }

    uint a = read_imageui(voronoi, sampler, (int2)(gid0 + 0, gid1 + 0));
    uint b = read_imageui(voronoi, sampler, (int2)(gid0 + 0, gid1 - 1));
    uint c = read_imageui(voronoi, sampler, (int2)(gid0 - 1, gid1 + 0));
    uint d = read_imageui(voronoi, sampler, (int2)(gid0 - 1, gid1 - 1));

    int diff = 1;

    if (b != a || b != c || b != d)
    {
        diff++;
    }

    if (c != a || c != d)
    {
        diff++;
    }

    if (d != a)
    {
        diff++;
    }

    uint4 emark = (diff == 2) ? (uint4)(1) : (uint4)(0);
    uint4 vmark = (diff >  2) ? (uint4)(1) : (uint4)(0);

    write_imageui(edge_marks,   (int2)(gid0, gid1), emark);
    write_imageui(vertex_marks, (int2)(gid0, gid1), vmark);
}
