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

#ifndef CORRIDORMAP_RENDER_INTERFACE_H_
#define CORRIDORMAP_RENDER_INTERFACE_H_

#include <CL/opencl.h>

namespace corridormap { struct Render_Vertex; }
namespace corridormap { class Memory; }

namespace corridormap {

// abtract rendering backend interface.
class Renderer
{
public:
    // initialization parameters.
    struct Parameters
    {
        // width of render target.
        unsigned render_target_width;
        // height of render target.
        unsigned render_target_height;
        // orthographic projection bounding box min.
        float min[2];
        // orthographic projection bounding box max.
        float max[2];
        // depth is in [0, far_plane] range.
        float far_plane;
    };

    // opencl context and device for this renderer's GPU.
    struct Opencl_Shared
    {
        cl_platform_id  platform;
        cl_device_id    device;
        cl_context      context;
    };

    Parameters params;

    virtual ~Renderer() {}

    // initialize renderer. returns false on failure.
    virtual bool initialize(Parameters params, Memory* scratch_memory) = 0;

    // begin scene. must be called before any calls to draw.
    virtual void begin() = 0;
    // draw mesh with uniform color. length of vertices array is tri_count*3.
    virtual void draw(const Render_Vertex* vertices, unsigned tri_count, unsigned color) = 0;
    // end scene. must be called after all calls to draw.
    virtual void end() = 0;

    // copy render target from video memory (only used when features are detected on cpu).
    virtual void read_pixels(unsigned char* destination) = 0;

    // creates shared opencl context.
    virtual Opencl_Shared create_opencl_shared() = 0;
    // creates opencl memory object shared with rendered backbuffer.
    virtual cl_mem share_pixels(cl_context shared_context, cl_mem_flags flags, cl_int* error_code) = 0;
    // acquires opencl/opengl shared object.
    virtual cl_int acquire_shared(cl_command_queue queue, cl_mem object) = 0;
    // releases opencl/opengl shared object.
    virtual cl_int release_shared(cl_command_queue queue, cl_mem object) = 0;
};

}

#endif
