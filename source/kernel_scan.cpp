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

const char* kernel_scan_reduce_source = \

"const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE|CLK_ADDRESS_CLAMP_TO_EDGE|CLK_FILTER_NEAREST;                    \n"
"                                                                                                                       \n"
"inline uint value1d(read_only image2d_t img, uint idx)                                                                 \n"
"{                                                                                                                      \n"
"    size_t w = get_image_width(img);                                                                                   \n"
"    int2 uv = (int2)(idx % w, idx / w);                                                                                \n"
"    return read_imageui(img, sampler, uv).s0;                                                                          \n"
"}                                                                                                                      \n"
"                                                                                                                       \n"
"kernel void run(read_only image2d_t image, global uint* global_sums, local uint* local_sums, const uint pixel_count)   \n"
"{                                                                                                                      \n"
"    size_t gid = get_global_id(0);                                                                                     \n"
"    size_t lid = get_local_id(0);                                                                                      \n"
"    size_t bid = get_group_id(0);                                                                                      \n"
"                                                                                                                       \n"
"    size_t block_size = get_local_size(0);                                                                             \n"
"    size_t elements_per_block = pixel_count / (2 * block_size);                                                        \n"
"                                                                                                                       \n"
"    uint sum = 0;                                                                                                      \n"
"    uint base = bid * elements_per_block;                                                                              \n"
"                                                                                                                       \n"
"    for (uint i = lid; i < elements_per_block; i += block_size)                                                        \n"
"    {                                                                                                                  \n"
"        uint idx = base + i;                                                                                           \n"
"                                                                                                                       \n"
"        if (idx < pixel_count)                                                                                         \n"
"        {                                                                                                              \n"
"            sum += value1d(image, idx);                                                                                \n"
"        }                                                                                                              \n"
"    }                                                                                                                  \n"
"                                                                                                                       \n"
"    local_sums[lid] = sum;                                                                                             \n"
"                                                                                                                       \n"
"    barrier(CLK_LOCAL_MEM_FENCE);                                                                                      \n"
"                                                                                                                       \n"
"    for (uint i = block_size >> 1; i > 0; i >>= 1)                                                                     \n"
"    {                                                                                                                  \n"
"        if (lid < i)                                                                                                   \n"
"        {                                                                                                              \n"
"            local_sums[lid] += local_sums[lid + i];                                                                    \n"
"        }                                                                                                              \n"
"                                                                                                                       \n"
"        barrier(CLK_LOCAL_MEM_FENCE);                                                                                  \n"
"    }                                                                                                                  \n"
"                                                                                                                       \n"
"    if (lid == 0)                                                                                                      \n"
"    {                                                                                                                  \n"
"        global_sums[bid] = local_sums[0];                                                                              \n"
"    }                                                                                                                  \n"
"}                                                                                                                      \n";

}
