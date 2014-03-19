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
        float min[3];
        // orthographic projection bounding box max.
        float max[3];
    };

    // mesh primitive type.
    enum primitive_type
    {
        primitive_type_fan = 0,
        primitive_type_stripe,
        primitive_type_list,
    };

    virtual ~renderer() {}

    // initialize renderer.
    virtual void initialize(renderer_parameters params) = 0;
    // release resources.
    virtual void finalize() = 0;
    // begin scene.
    virtual void begin() = 0;
    // draw mesh with uniform color. length of vertices array is count*3.
    virtual void draw(const float* vertices, unsigned count, unsigned color, mesh_primitive_type primitive_type) = 0;
    // end scene.
    virtual void end() = 0;
    // copy render target from video memory.
    virtual void read_pixels(unsigned char* destination) = 0;
};

}

#endif
