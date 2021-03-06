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

const char* kernel_store_edge_obstacle_ids_source = \

"const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE|CLK_ADDRESS_CLAMP_TO_EDGE|CLK_FILTER_NEAREST;     \n"
"                                                                                                        \n"
"uint pack_color(float4 color)                                                                           \n"
"{                                                                                                       \n"
"    return (uint)(255.f * color.s0) << 24 |                                                             \n"
"           (uint)(255.f * color.s1) << 16 |                                                             \n"
"           (uint)(255.f * color.s2) << 8  |                                                             \n"
"           (uint)(255.f * color.s3) << 0  ;                                                             \n"
"}                                                                                                       \n"
"                                                                                                        \n"
"kernel void run(read_only image2d_t voronoi, read_only image2d_t edge_marks,                            \n"
"                global uint* indices, global uint* side_1_ids, global uint* side_2_ids)                 \n"
"{                                                                                                       \n"
"    size_t gid = get_global_id(0);                                                                      \n"
"    size_t width = get_image_width(voronoi);                                                            \n"
"                                                                                                        \n"
"    uint idx = indices[gid];                                                                            \n"
"    int2 uv = (int2)(idx % width, idx / width);                                                         \n"
"                                                                                                        \n"
"    uint a = pack_color(read_imagef(voronoi, sampler, uv + (int2)(-1, -1)));                            \n"
"    uint b = pack_color(read_imagef(voronoi, sampler, uv + (int2)(+0, -1)));                            \n"
"    uint c = pack_color(read_imagef(voronoi, sampler, uv + (int2)(-1, +0)));                            \n"
"    uint d = pack_color(read_imagef(voronoi, sampler, uv + (int2)(+0, +0)));                            \n"
"                                                                                                        \n"
"    uint ma = read_imageui(edge_marks, sampler, uv + (int2)(-1, -1)).s0 > 0 ? 1 : 0;                    \n"
"    uint mb = read_imageui(edge_marks, sampler, uv + (int2)(+0, -1)).s0 > 0 ? 1 : 0;                    \n"
"    uint mc = read_imageui(edge_marks, sampler, uv + (int2)(-1, +0)).s0 > 0 ? 1 : 0;                    \n"
"    uint md = read_imageui(edge_marks, sampler, uv + (int2)(+0, +0)).s0 > 0 ? 1 : 0;                    \n"
"                                                                                                        \n"
"    side_1_ids[gid] = a;                                                                                \n"
"    side_2_ids[gid] = (a != b) ? b : ((a != c) ? c : d);                                                \n"
"}                                                                                                       \n";

}
