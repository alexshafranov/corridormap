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

// Example OpenGL implementation of the render interface.
// OpenGL header has to be included before including this file.

#ifndef CORRIDORMAP_RENDER_GL_H_
#define CORRIDORMAP_RENDER_GL_H_

#include <stdio.h>
#include "corridormap/types.h"
#include "corridormap/render_interface.h"

namespace corridormap {

static const char* vertex_shader =
"uniform mat4 wvp;                                  \n"
"in vec3 position;                                  \n"
"                                                   \n"
"void main()                                        \n"
"{                                                  \n"
"    gl_Position = wvp * vec4(position.xyz, 1.0);   \n"
"}                                                  \n";

static const char* fragment_shader =
"uniform vec3 const_color;                          \n"
"out vec4 out_color;                                \n"
"                                                   \n"
"void main()                                        \n"
"{                                                  \n"
"    out_color = vec4(const_color.rgb, 1.0);        \n"
"}                                                  \n";

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
}

class renderer_gl : public renderer
{
public:

    virtual bool initialize(renderer::parameters params)
    {
        _params = params;

        glGenFramebuffers(1, &_frame_buffer);
        glGenTextures(2, _output_textures);

        // initialize output textures
        {
            glBindTexture(GL_TEXTURE_2D, _output_textures[0]);
            glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, params.render_target_width, params.render_target_height);

            if (glGetError() != GL_NO_ERROR)
            {
                return false;
            }

            glBindTexture(GL_TEXTURE_2D, _output_textures[1]);
            glTexStorage2D(GL_TEXTURE_2D, 1, GL_DEPTH_COMPONENT32F, params.render_target_width, params.render_target_height);

            if (glGetError() != GL_NO_ERROR)
            {
                return false;
            }

            glBindTexture(GL_TEXTURE_2D, 0);
        }

        // attach textures to the frame buffer object.
        {
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _frame_buffer);

            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _output_textures[0], 0);

            if (glGetError() != GL_NO_ERROR)
            {
                return false;
            }

            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, _output_textures[1], 0);

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

        // initialize shaders and shader attributes.
        {
            _program = glCreateProgram();
            _vertex_shader = glCreateShader(GL_VERTEX_SHADER);
            _fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

            glShaderSource(_vertex_shader, 1, &vertex_shader, 0);
            glShaderSource(_fragment_shader, 1, &fragment_shader, 0);

            GLint status;
            glCompileShader(_vertex_shader);
            glCompileShader(_fragment_shader);

            glGetShaderiv(_vertex_shader, GL_COMPILE_STATUS, &status);

            if (status != GL_TRUE)
            {
                print_shader_log(_vertex_shader);
                return false;
            }

            glGetShaderiv(_fragment_shader, GL_COMPILE_STATUS, &status);

            if (status != GL_TRUE)
            {
                print_shader_log(_fragment_shader);
                return false;
            }

            glAttachShader(_program, _vertex_shader);
            glAttachShader(_program, _fragment_shader);

            glBindAttribLocation(_program, 0, "position");

            glLinkProgram(_program);

            if (glGetError() != GL_NO_ERROR)
            {
                return false;
            }

            _wvp_location = glGetUniformLocation(_program, "wvp");
            _color_location = glGetUniformLocation(_program, "const_color");
        }

        // setup orthographic projection. projection is left-haded, camera is in zero looking in +z direction.
        {
            float l = _params.min[0];
            float r = _params.max[0];
            float b = _params.min[1];
            float t = _params.max[1];
            float n = 0.f;
            float f = _params.far_plane;

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

        return true;
    }

    virtual void finalize()
    {
        glDeleteBuffers(1, &_vertex_buffer);
        glDeleteVertexArrays(1, &_vertex_array);
        glDeleteTextures(2, _output_textures);
        glDeleteFramebuffers(1, &_frame_buffer);
    }

    virtual void begin()
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _frame_buffer);
        glViewport(0, 0, _params.render_target_width, _params.render_target_height);
        glClearColor(0.f, 1.f, 0.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(_program);
        glUniformMatrix4fv(_wvp_location, 1, GL_FALSE, _projection);

        glEnable(GL_CULL_FACE);
        glFrontFace(GL_CCW);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
    }

    virtual void draw(const vertex* vertices, unsigned tri_count, unsigned color)
    {
        // set color for fragment shader.
        glUniform3f(
            _color_location,
            ((color & 0x000000ff) >>  0)/255.f,
            ((color & 0x0000ff00) >>  8)/255.f,
            ((color & 0x00ff0000) >> 16)/255.f);

        // upload vertices.
        glBindVertexArray(_vertex_array);
        glBindBuffer(GL_ARRAY_BUFFER, _vertex_buffer);
        glBufferData(GL_ARRAY_BUFFER, tri_count*3*sizeof(vertex), vertices, GL_STREAM_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), 0);

        // render.
        glDrawArrays(GL_TRIANGLES, 0, tri_count * 3);

        // cleanup.
        glDisableVertexAttribArray(0);
    }

    virtual void end()
    {
        glUseProgram(0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    }

    virtual void read_pixels(unsigned char* /*destination*/)
    {
    }

    void blit_frame_buffer(int width, int height)
    {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, _frame_buffer);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glBlitFramebuffer(0, 0, _params.render_target_width, _params.render_target_height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_LINEAR);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    }

private:
    renderer::parameters _params;

    GLuint _frame_buffer;
    GLuint _output_textures[2];
    GLuint _vertex_array;
    GLuint _vertex_buffer;
    GLuint _program;
    GLuint _vertex_shader;
    GLuint _fragment_shader;
    GLint  _wvp_location;
    GLint  _color_location;

    GLfloat _projection[4*4];
};

}

#endif
