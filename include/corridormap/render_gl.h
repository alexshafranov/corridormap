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

// opengl 3 implementation of the render interface.

#ifndef CORRIDORMAP_RENDER_GL_H_
#define CORRIDORMAP_RENDER_GL_H_

#if defined CORRIDORMAP_ENABLE_OPENCL_SHARING
    #define CORRIDORMAP_OPENCL_SHARING 1
#endif

#include <stdio.h>
#include "corridormap/build_types.h"
#include "corridormap/memory.h"
#include "corridormap/render_interface.h"

namespace corridormap {

static const char* vertex_shader =
"#version 330                                       \n"
"uniform mat4 wvp;                                  \n"
"in vec3 position;                                  \n"
"                                                   \n"
"void main()                                        \n"
"{                                                  \n"
"    gl_Position = wvp * vec4(position.xyz, 1.0);   \n"
"}                                                  \n";

static const char* fragment_shader =
"#version 330                                       \n"
"uniform vec4 const_color;                          \n"
"out vec4 out_color;                                \n"
"                                                   \n"
"void main()                                        \n"
"{                                                  \n"
"    out_color = const_color;                       \n"
"}                                                  \n";

static const char* debug_quad_vertex_shader =
"#version 330                                       \n"
"in vec3 position;                                  \n"
"out vec2 uv;                                       \n"
"                                                   \n"
"void main()                                        \n"
"{                                                  \n"
"   uv = vec2(position.x, -position.y) * 0.5 + 0.5; \n"
"   gl_Position = vec4(position.xyz, 1.0);          \n"
"}                                                  \n";

static const char* debug_quad_fragment_shader =
"#version 330                                   \n"
"uniform sampler2D t;                           \n"
"in vec2 uv;                                    \n"
"out vec4 out_color;                            \n"
"                                               \n"
"void main()                                    \n"
"{                                              \n"
"   out_color = vec4(texture(t, uv).rgb, 1.0);  \n"
"}                                              \n";

namespace
{
    void print_shader_log(GLuint shader)
    {
        const int max_buffer_size = 4096;

        char buffer[max_buffer_size + 1];
        int length = 0;

        glGetShaderInfoLog(shader, max_buffer_size, &length, buffer);

        if (length > max_buffer_size)
        {
            length = max_buffer_size;
        }

        buffer[length] = '\0';

        fprintf(stderr, "compilation log:\n%s\n", buffer);
    }

    struct shader
    {
        GLuint program;
        GLuint vertex_shader;
        GLuint fragment_shader;
    };

    shader create_shader(const char* vs_source, const char* fs_source)
    {
        shader result;
        memset(&result, 0, sizeof(shader));

        GLuint program = glCreateProgram();
        GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
        GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

        glShaderSource(vertex_shader, 1, &vs_source, 0);
        glShaderSource(fragment_shader, 1, &fs_source, 0);

        GLint status;
        glCompileShader(vertex_shader);
        glCompileShader(fragment_shader);

        glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &status);

        if (status != GL_TRUE)
        {
            print_shader_log(vertex_shader);
            return result;
        }

        glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &status);

        if (status != GL_TRUE)
        {
            print_shader_log(fragment_shader);
            return result;
        }

        glAttachShader(program, vertex_shader);
        glAttachShader(program, fragment_shader);

        glBindAttribLocation(program, 0, "position");

        glLinkProgram(program);

        if (glGetError() != GL_NO_ERROR)
        {
            return result;
        }

        result.program = program;
        result.vertex_shader = vertex_shader;
        result.fragment_shader = fragment_shader;

        return result;
    }

    void destroy_shader(shader& s)
    {
        glDeleteShader(s.vertex_shader);
        glDeleteShader(s.fragment_shader);
        glDeleteShader(s.program);
    }
}

class Renderer_GL : public Renderer
{
public:

    Renderer_GL()
        : _scratch_memory(0)
        , _frame_buffer(0)
        , _color_buffer_texture(0)
        , _depth_buffer_texture(0)
        , _vertex_array(0)
        , _vertex_buffer(0)
    {
        memset(&_draw_shader, 0, sizeof(shader));
        memset(&_debug_quad_shader, 0, sizeof(shader));
    }

    virtual ~Renderer_GL()
    {
        destroy_shader(_draw_shader);
        destroy_shader(_debug_quad_shader);
        glDeleteBuffers(1, &_vertex_buffer);
        glDeleteVertexArrays(1, &_vertex_array);
        glDeleteTextures(1, &_depth_buffer_texture);
        glDeleteTextures(1, &_color_buffer_texture);
        glDeleteFramebuffers(1, &_frame_buffer);
    }

    virtual bool initialize(Renderer::Parameters params_, Memory* scratch_memory)
    {
        params = params_;
        _scratch_memory = scratch_memory;

        glGenFramebuffers(1, &_frame_buffer);
        glGenTextures(1, &_color_buffer_texture);
        glGenTextures(1, &_depth_buffer_texture);

        // initialize output textures
        {
            glBindTexture(GL_TEXTURE_2D, _color_buffer_texture);
            glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, params.render_target_width, params.render_target_height);

            if (glGetError() != GL_NO_ERROR)
            {
                return false;
            }

            glBindTexture(GL_TEXTURE_2D, _depth_buffer_texture);
            glTexStorage2D(GL_TEXTURE_2D, 1, GL_DEPTH24_STENCIL8, params.render_target_width, params.render_target_height);

            if (glGetError() != GL_NO_ERROR)
            {
                return false;
            }

            glBindTexture(GL_TEXTURE_2D, 0);
        }

        // attach textures to the frame buffer object.
        {
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _frame_buffer);

            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _color_buffer_texture, 0);

            if (glGetError() != GL_NO_ERROR)
            {
                return false;
            }

            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, _depth_buffer_texture, 0);

            if (glGetError() != GL_NO_ERROR)
            {
                return false;
            }

            if (glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            {
                return false;
            }

            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        }

        glGenVertexArrays(1, &_vertex_array);
        glGenBuffers(1, &_vertex_buffer);

        // initialize shaders and shader parameters.
        _draw_shader = create_shader(vertex_shader, fragment_shader);
        _wvp_location = glGetUniformLocation(_draw_shader.program, "wvp");
        _color_location = glGetUniformLocation(_draw_shader.program, "const_color");

        // setup orthographic projection. projection is left-haded, camera is in zero looking in +z direction.
        {
            float l = params.min[0];
            float r = params.max[0];
            float b = params.min[1];
            float t = params.max[1];
            float n = 0.f;
            float f = params.far_plane;

            // matrix is stored in a column major order.
            _projection[0*4 + 0] = 2.f / (r - l);
            _projection[0*4 + 1] = 0.f;
            _projection[0*4 + 2] = 0.f;
            _projection[0*4 + 3] = 0.f;

            _projection[1*4 + 0] = 0.f;
            _projection[1*4 + 1] = 2.f / (t - b);
            _projection[1*4 + 2] = 0.f;
            _projection[1*4 + 3] = 0.f;

            _projection[2*4 + 0] = 0.f;
            _projection[2*4 + 1] = 0.f;
            _projection[2*4 + 2] = 2.f / (f - n);
            _projection[2*4 + 3] = 0.f;

            _projection[3*4 + 0] = (l + r) / (l - r);
            _projection[3*4 + 1] = (t + b) / (b - t);
            _projection[3*4 + 2] = (n + f) / (n - f);
            _projection[3*4 + 3] = 1.f;
        }

        // initialize debug shaders.
        _debug_quad_shader = create_shader(debug_quad_vertex_shader, debug_quad_fragment_shader);

        return true;
    }

    virtual void begin()
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _frame_buffer);

        glViewport(0, 0, params.render_target_width, params.render_target_height);
        glClearColor(1.f, 1.f, 1.f, 1.f);
        glClearDepth(1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(_draw_shader.program);
        glUniformMatrix4fv(_wvp_location, 1, GL_FALSE, _projection);

        glEnable(GL_CULL_FACE);
        glFrontFace(GL_CCW);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
    }

    virtual void draw(const Render_Vertex* vertices, unsigned tri_count, unsigned color)
    {
        // set color for fragment shader.
        glUniform4f(
            _color_location,
            ((color & 0xff000000) >> 24)/255.f,
            ((color & 0x00ff0000) >> 16)/255.f,
            ((color & 0x0000ff00) >>  8)/255.f,
            ((color & 0x000000ff) >>  0)/255.f);

        draw_array(vertices, tri_count);
    }

    void draw_array(const Render_Vertex* vertices, unsigned tri_count)
    {
        // upload vertices.
        glBindVertexArray(_vertex_array);
        glBindBuffer(GL_ARRAY_BUFFER, _vertex_buffer);
        glBufferData(GL_ARRAY_BUFFER, tri_count*3*sizeof(Render_Vertex), vertices, GL_STREAM_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Render_Vertex), 0);

        // render.
        glDrawArrays(GL_TRIANGLES, 0, tri_count * 3);

        // cleanup.
        glDisableVertexAttribArray(0);
    }

    virtual void end()
    {
        glUseProgram(0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glFinish();
    }

    virtual void read_pixels(unsigned char* destination)
    {
        glBindTexture(GL_TEXTURE_2D, _color_buffer_texture);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, destination);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void blit_frame_buffer(int width, int height)
    {
        // enable back-buffer.
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

        glViewport(0, 0, width, height);
        glClearColor(1.f, 1.f, 1.f, 1.f);
        glClearDepth(1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // state.
        glEnable(GL_CULL_FACE);
        glFrontFace(GL_CCW);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);

        // texture.
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, _color_buffer_texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        Render_Vertex quad[] =
        {
            {-1.f, -1.f,  0.f},
            { 1.f, -1.f,  0.f},
            {-1.f,  1.f,  0.f},
            {-1.f,  1.f,  0.f},
            { 1.f, -1.f,  0.f},
            { 1.f,  1.f,  0.f},
        };

        glUseProgram(_debug_quad_shader.program);
        draw_array(quad, 2);

        glBindTexture(GL_TEXTURE_2D, 0);
        glUseProgram(0);
    }

    virtual Opencl_Shared create_opencl_shared()
    {
        Opencl_Shared result;
        memset(&result, 0, sizeof(result));

        #if CORRIDORMAP_OPENCL_SHARING
            cl_int error_code;

            clGetGLContextInfoKHR_fn pclGetGLContextInfoKHR = reinterpret_cast<clGetGLContextInfoKHR_fn>(clGetExtensionFunctionAddress("clGetGLContextInfoKHR"));

            // find platform and device which could be shared with opengl and create opencl context.

            cl_uint num_platforms;
            clGetPlatformIDs(0, 0, &num_platforms);

            cl_platform_id* platforms = allocate<cl_platform_id>(_scratch_memory, num_platforms);
            clGetPlatformIDs(num_platforms, platforms, 0);

            for (cl_uint i = 0; i < num_platforms; ++i)
            {
                cl_platform_id platform = platforms[i];

                #ifdef _WIN32
                    cl_context_properties properties[] =
                    {
                        CL_GL_CONTEXT_KHR, reinterpret_cast<cl_context_properties>(wglGetCurrentContext()),
                        CL_WGL_HDC_KHR, reinterpret_cast<cl_context_properties>(wglGetCurrentDC()),
                        CL_CONTEXT_PLATFORM, reinterpret_cast<cl_context_properties>(platform),
                        0,
                    };
                #endif

                cl_device_id device;
                error_code = pclGetGLContextInfoKHR(properties, CL_CURRENT_DEVICE_FOR_GL_CONTEXT_KHR, sizeof(cl_device_id), &device, 0);

                if (error_code != CL_SUCCESS)
                {
                    continue;
                }

                cl_context context = clCreateContextFromType(properties, CL_DEVICE_TYPE_GPU, 0, 0, &error_code);

                if (error_code != CL_SUCCESS)
                {
                    continue;
                }

                result.platform = platform;
                result.device = device;
                result.context = context;

                return result;
            }
        #endif

        return result;
    }

    virtual cl_mem share_pixels(cl_context shared_context, cl_mem_flags flags, cl_int* error_code)
    {
        #if CORRIDORMAP_OPENCL_SHARING
            return clCreateFromGLTexture2D(shared_context, flags, GL_TEXTURE_2D, 0, _color_buffer_texture, error_code);
        #else
            (void)shared_context;
            (void)flags;
            (void)error_code;
            return 0;
        #endif
    }

    virtual cl_int acquire_shared(cl_command_queue queue, cl_mem object)
    {
        #if CORRIDORMAP_OPENCL_SHARING
            return clEnqueueAcquireGLObjects(queue, 1, &object, 0, 0, 0);
        #else
            (void)queue;
            (void)object;
            return 0;
        #endif
    }

    virtual cl_int release_shared(cl_command_queue queue, cl_mem object)
    {
        #if CORRIDORMAP_OPENCL_SHARING
            return clEnqueueReleaseGLObjects(queue, 1, &object, 0, 0, 0);
        #else
            (void)queue;
            (void)object;
            return 0;
        #endif
    }

private:
    Memory* _scratch_memory;

    GLuint _frame_buffer;
    GLuint _color_buffer_texture;
    GLuint _depth_buffer_texture;

    GLuint _vertex_array;
    GLuint _vertex_buffer;

    shader _draw_shader;
    GLint  _wvp_location;
    GLint  _color_location;

    shader _debug_quad_shader;

    GLfloat _projection[4*4];
};

}

#endif
