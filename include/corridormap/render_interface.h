#ifndef CORRIDORMAP_RENDER_INTERFACE_H_
#define CORRIDORMAP_RENDER_INTERFACE_H_

namespace corridormap {

struct vertex
{
    float x;
    float y;
    float z;
};

// parameters for rendering interface initialization.
struct renderer_parameters
{
    // width of render target.
    unsigned render_target_width;
    // height of render target.
    unsigned render_target_height;
    // orthographic projection bounding box min.
    vertex min;
    // orthographic projection bounding box max.
    vertex max;
};

// mesh primitive type.
enum renderer_primitive_type
{
    primitive_type_fan = 0,
    primitive_type_stripe,
    primitive_type_list,
};

// abtract rendering backend interface.
class renderer
{
public:
    virtual ~renderer() {}

    // initialize renderer.
    virtual void initialize(renderer_parameters params) = 0;
    // release resources.
    virtual void finalize() = 0;
    // begin scene.
    virtual void begin() = 0;
    // draw mesh with uniform color.
    virtual void draw(const vertex* vertices, unsigned count, unsigned color, renderer_primitive_type primitive_type) = 0;
    // end scene.
    virtual void end() = 0;
    // copy render target from vram.
    virtual void read_pixels(unsigned char* destination) = 0;
};

}

#endif
