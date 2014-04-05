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

namespace corridormap {

const char* kernel_mark_poi_source = \

"const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE|CLK_ADDRESS_CLAMP|CLK_FILTER_NEAREST;        \n"
"                                                                                                   \n"
"uint pack_color(uint4 color)                                                                       \n"
"{                                                                                                  \n"
"    uint result = 0u;                                                                              \n"
"    result |= 0xff000000 & color.s0;                                                               \n"
"    result |= 0x00ff0000 & color.s1;                                                               \n"
"    result |= 0x0000ff00 & color.s2;                                                               \n"
"    result |= 0x000000ff & color.s3;                                                               \n"
"    return result;                                                                                 \n"
"}                                                                                                  \n"
"                                                                                                   \n"
"kernel void mark_poi(                                                                              \n"
"    read_only  image2d_t voronoi,                                                                  \n"
"    write_only image2d_t vertex_marks,                                                             \n"
"    write_only image2d_t edge_marks)                                                               \n"
"{                                                                                                  \n"
"    int gid0 = get_global_id(0);                                                                   \n"
"    int gid1 = get_global_id(1);                                                                   \n"
"                                                                                                   \n"
"    uint a = pack_color(read_imageui(voronoi, sampler, (int2)(gid0 + 0, gid1 + 0)));               \n"
"    uint b = pack_color(read_imageui(voronoi, sampler, (int2)(gid0 + 0, gid1 - 1)));               \n"
"    uint c = pack_color(read_imageui(voronoi, sampler, (int2)(gid0 - 1, gid1 + 0)));               \n"
"    uint d = pack_color(read_imageui(voronoi, sampler, (int2)(gid0 - 1, gid1 - 1)));               \n"
"                                                                                                   \n"
"    int diff = 1;                                                                                  \n"
"                                                                                                   \n"
"    if (b != a || b != c || b != d)                                                                \n"
"    {                                                                                              \n"
"        diff++;                                                                                    \n"
"    }                                                                                              \n"
"                                                                                                   \n"
"    if (c != a || c != d)                                                                          \n"
"    {                                                                                              \n"
"        diff++;                                                                                    \n"
"    }                                                                                              \n"
"                                                                                                   \n"
"    if (d != a)                                                                                    \n"
"    {                                                                                              \n"
"        diff++;                                                                                    \n"
"    }                                                                                              \n"
"                                                                                                   \n"
"    uint4 emark = (diff == 2) ? (uint4)(1, 1, 1, 1) : (uint4)(0, 0, 0, 0);                         \n"
"    uint4 vmark = (diff >  2) ? (uint4)(1, 1, 1, 1) : (uint4)(0, 0, 0, 0);                         \n"
"                                                                                                   \n"
"    write_imageui(edge_marks,   (int2)(gid0, gid1), emark);                                        \n"
"    write_imageui(vertex_marks, (int2)(gid0, gid1), vmark);                                        \n"
"}                                                                                                  \n";

const char* kernel_mark_poi_debug_source = \

"const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE|CLK_ADDRESS_CLAMP|CLK_FILTER_NEAREST;        \n"
"                                                                                                   \n"
"kernel void mark_poi_debug(                                                                        \n"
"    read_only  image2d_t marks,                                                                    \n"
"    write_only image2d_t voronoi,                                                                  \n"
"    const uint color)                                                                              \n"
"{                                                                                                  \n"
"    int gid0 = get_global_id(0);                                                                   \n"
"    int gid1 = get_global_id(1);                                                                   \n"
"                                                                                                   \n"
"    uint m = read_imageui(marks, sampler, (int2)(gid0, gid1)).s0;                                  \n"
"    uint4 out_color;                                                                               \n"
"    out_color.s0 = color & 0x00ff0000;                                                             \n"
"    out_color.s1 = color & 0x0000ff00;                                                             \n"
"    out_color.s2 = color & 0x000000ff;                                                             \n"
"    out_color.s3 = color & 0xff000000;                                                             \n"
"                                                                                                   \n"
"    if (m)                                                                                         \n"
"    {                                                                                              \n"
"        write_imageui(voronoi, (int2)(gid0, gid1), out_color);                                     \n"
"    }                                                                                              \n"
"}                                                                                                  \n";

}
