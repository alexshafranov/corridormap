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

namespace corridormap { struct vertex; }
namespace corridormap { class memory; }

namespace corridormap {

// abtract rendering backend interface.
class renderer
{
public:
    // initialization parameters.
    struct parameters
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
    struct opencl_shared
    {
        cl_platform_id  platform;
        cl_device_id    device;
        cl_context      context;
    };

    virtual ~renderer() {}

    // initialize renderer. returns false on failure.
    virtual bool initialize(parameters params, memory* scratch_memory) = 0;
    // release resources.
    virtual void finalize() = 0;
    // begin scene.
    virtual void begin() = 0;
    // draw mesh with uniform color. length of vertices array is tri_count*3.
    virtual void draw(const vertex* vertices, unsigned tri_count, unsigned color) = 0;
    // end scene.
    virtual void end() = 0;
    // copy render target from video memory.
    virtual void read_pixels(unsigned char* destination) = 0;

    // create shared opencl context.
    virtual opencl_shared create_opencl_shared() = 0;
    // creates opencl memory object shared with rendered backbuffer.
    virtual cl_mem share_pixels(cl_context shared_context, cl_mem_flags flags, cl_int* error_code) = 0;
    // acquire opencl/opengl shared object.
    virtual cl_int acquire_shared(cl_command_queue queue, cl_mem object) = 0;
    // release opencl/opengl shared object.
    virtual cl_int release_shared(cl_command_queue queue, cl_mem object) = 0;
};

}

#endif
