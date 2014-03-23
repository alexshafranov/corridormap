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

// OpenGL implementation of the render interface.
// OpenGL header has to be included before including this file.

#ifndef CORRIDORMAP_RENDER_GL_H_
#define CORRIDORMAP_RENDER_GL_H_

#include "corridormap/render_interface.h"

namespace corridormap {

static const char* vertex_shader =
"in vec3 position;                                  \n"
"                                                   \n"
"void main()                                        \n"
"{                                                  \n"
"    gl_Position = wvp * vec4(position.xyz, 1.0);   \n"
"}                                                  \n";

static const char* fragment_shader =
"out vec4 out_color;                                \n"
"                                                   \n"
"void main()                                        \n"
"{                                                  \n"
"    out_color = vec4(1.0, 0.0, 1.0, 1.0);          \n"
"}                                                  \n";

class renderer_gl : public renderer
{
public:
    virtual void initialize(renderer::parameters /*params*/)
    {
        glGenVertexArrays(1, &_vertex_array);
        glGenBuffers(1, &_vertex_buffer);

        _program = glCreateProgram();
        _vertex_shader = glCreateShader(GL_VERTEX_SHADER);
        _fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(_vertex_shader, 1, &vertex_shader, 0);
        glShaderSource(_fragment_shader, 1, &fragment_shader, 0);
        glCompileShader(_vertex_shader);
        glCompileShader(_fragment_shader);
        glAttachShader(_program, _vertex_shader);
        glAttachShader(_program, _fragment_shader);
        glBindAttribLocation(_program, 0, "position");
    }

    virtual void finalize()
    {
        glDeleteBuffers(1, &_vertex_buffer);
        glDeleteVertexArrays(1, &_vertex_array);
    }

    virtual void begin()
    {
    }

    virtual void draw(const float* /*vertices*/, unsigned /*count*/, unsigned /*color*/, renderer::primitive_type /*pt*/)
    {
    }

    virtual void end()
    {
    }

    virtual void read_pixels(unsigned char* /*destination*/)
    {
    }

private:
    GLuint _vertex_array;
    GLuint _vertex_buffer;
    GLuint _program;
    GLuint _vertex_shader;
    GLuint _fragment_shader;
};

}

#endif
