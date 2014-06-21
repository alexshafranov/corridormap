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

const char* kernel_mark_features_source = \

"const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE|CLK_ADDRESS_CLAMP_TO_EDGE|CLK_FILTER_NEAREST;    \n"
"                                                                                                       \n"
"uint pack_color(float4 color)                                                                          \n"
"{                                                                                                      \n"
"    return (uint)(255.f * color.s0) << 24 |                                                            \n"
"           (uint)(255.f * color.s1) << 16 |                                                            \n"
"           (uint)(255.f * color.s2) << 8  |                                                            \n"
"           (uint)(255.f * color.s3) << 0  ;                                                            \n"
"}                                                                                                      \n"
"                                                                                                       \n"
"kernel void run(                                                                                       \n"
"    read_only  image2d_t voronoi,                                                                      \n"
"    write_only image2d_t vertex_marks,                                                                 \n"
"    write_only image2d_t edge_marks)                                                                   \n"
"{                                                                                                      \n"
"    size_t gid0 = get_global_id(0);                                                                    \n"
"    size_t gid1 = get_global_id(1);                                                                    \n"
"                                                                                                       \n"
"    int2 uv = (int2)(gid0, gid1);                                                                      \n"
"                                                                                                       \n"
"    uint a = pack_color(read_imagef(voronoi, sampler, uv + (int2)(-1, -1)));                           \n"
"    uint b = pack_color(read_imagef(voronoi, sampler, uv + (int2)(+0, -1)));                           \n"
"    uint c = pack_color(read_imagef(voronoi, sampler, uv + (int2)(-1, +0)));                           \n"
"    uint d = pack_color(read_imagef(voronoi, sampler, uv + (int2)(+0, +0)));                           \n"
"                                                                                                       \n"
"    int diff = 1;                                                                                      \n"
"                                                                                                       \n"
"    if (b != a)                                                                                        \n"
"    {                                                                                                  \n"
"        diff++;                                                                                        \n"
"    }                                                                                                  \n"
"                                                                                                       \n"
"    if (c != a && c != b)                                                                              \n"
"    {                                                                                                  \n"
"        diff++;                                                                                        \n"
"    }                                                                                                  \n"
"                                                                                                       \n"
"    if (d != a && d != b && d != c)                                                                    \n"
"    {                                                                                                  \n"
"        diff++;                                                                                        \n"
"    }                                                                                                  \n"
"                                                                                                       \n"
"    uint4 vmark = (diff >  2) ? (uint4)(1, 1, 1, 1) : (uint4)(0, 0, 0, 0);                             \n"
"    uint4 emark = (diff == 2) ? (uint4)(1, 1, 1, 1) : (uint4)(0, 0, 0, 0);                             \n"
"                                                                                                       \n"
"    write_imageui(vertex_marks, (int2)(gid0, gid1), vmark);                                            \n"
"    write_imageui(edge_marks,   (int2)(gid0, gid1), emark);                                            \n"
"}                                                                                                      \n";

const char* kernel_mark_features_debug_source = \

"const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE|CLK_ADDRESS_CLAMP|CLK_FILTER_NEAREST;            \n"
"                                                                                                       \n"
"kernel void run(                                                                                       \n"
"    read_only  image2d_t marks,                                                                        \n"
"    write_only image2d_t voronoi,                                                                      \n"
"    const uint color,                                                                                  \n"
"    const int  border,                                                                                 \n"
"    const uint width,                                                                                  \n"
"    const uint height)                                                                                 \n"
"{                                                                                                      \n"
"    int gid0 = get_global_id(0);                                                                       \n"
"    int gid1 = get_global_id(1);                                                                       \n"
"                                                                                                       \n"
"    int2 coords = (int2)(gid0, gid1);                                                                  \n"
"                                                                                                       \n"
"    uint m = read_imageui(marks, sampler, coords).s0;                                                  \n"
"                                                                                                       \n"
"    float4 out_color;                                                                                  \n"
"    out_color.s0 = (color & 0x00ff0000) / 255.0;                                                       \n"
"    out_color.s1 = (color & 0x0000ff00) / 255.0;                                                       \n"
"    out_color.s2 = (color & 0x000000ff) / 255.0;                                                       \n"
"    out_color.s3 = (color & 0xff000000) / 255.0;                                                       \n"
"                                                                                                       \n"
"    if (m > 0)                                                                                         \n"
"    {                                                                                                  \n"
"        for (int x = -border; x <= border; ++x)                                                        \n"
"        {                                                                                              \n"
"            for (int y = -border; y <= border; ++y)                                                    \n"
"            {                                                                                          \n"
"                int2 c = coords + (int2)(x, y);                                                        \n"
"                                                                                                       \n"
"                if (c.x >= 0 && c.y >= 0 && c.x < width && c.y < height)                               \n"
"                {                                                                                      \n"
"                    write_imagef(voronoi, c, out_color);                                               \n"
"                }                                                                                      \n"
"            }                                                                                          \n"
"        }                                                                                              \n"
"    }                                                                                                  \n"
"}                                                                                                      \n";

}
