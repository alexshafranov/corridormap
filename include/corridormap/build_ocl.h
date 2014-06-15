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

#ifndef CORRIDORMAP_BUILD_OCL_H_
#define CORRIDORMAP_BUILD_OCL_H_

#include "corridormap/build_types.h"
#include "corridormap/build_ocl_types.h"
#include "corridormap/render_interface.h"

namespace corridormap { class Memory; }

namespace corridormap {

// inits library's opencl runtime from render interface shared context, creates opencl command queue.
Opencl_Runtime init_opencl_runtime(const Renderer::Opencl_Shared& shared);

// releases opencl objects.
void term_opencl_runtime(Opencl_Runtime& runtime);

// returns source code for the kernel with specified id.
const char* get_kernel_source(Kernel_Id id);

// creates and compiles library's opencl kernels.
Compilation_Status build_kernels(Opencl_Runtime& runtime);

// marks voronoi vertices and egdes in runtime.voronoi_vertices_img and voronoi_edges_img from voronoi_image.
cl_int mark_voronoi_features(Opencl_Runtime& runtime, cl_mem voronoi_image);

// draw marks back to original voronoi image.
cl_int debug_voronoi_features(Opencl_Runtime& runtime, cl_mem voronoi_image, cl_mem marks_image, unsigned int color, unsigned int border);

// compact voronoi features on gpu, storing results in runtime.voronoi_vertices_compacted_buf and runtime.voronoi_edges_compacted_buf buffers.
cl_int compact_voronoi_features(Opencl_Runtime& runtime);

// store obstacle ids (colors) for vertices and edge points in compact arrays.
cl_int store_obstacle_ids(Opencl_Runtime& runtime, cl_mem voronoi_image);

// copy computed data from opencl device memory.
cl_int transfer_voronoi_features(Opencl_Runtime& runtime, Voronoi_Features& features);

}

#endif
