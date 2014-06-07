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

#ifndef CORRIDORMAP_BUILD_VEC2_H_
#define CORRIDORMAP_BUILD_VEC2_H_

namespace corridormap {

inline vec2 make_vec2(float x, float y)
{
    vec2 result = { x, y };
    return result;
}

inline vec2 add(vec2 a, vec2 b)
{
    return make_vec2(a.x + b.x, a.y + b.y);
}

inline vec2 sub(vec2 a, vec2 b)
{
    return make_vec2(a.x - b.x, a.y - b.y);
}

inline vec2 mul(vec2 a, vec2 b)
{
    return make_vec2(a.x * b.x, a.y * b.y);
}

inline vec2 scale(vec2 a, float val)
{
    return make_vec2(a.x*val, a.y*val);
}

inline vec2 normalized(vec2 a)
{
    return scale(a, 1.f/sqrt(a.x*a.x + a.y*a.y));
}

inline float dot(vec2 a, vec2 b)
{
    return a.x*b.x + a.y*b.y;
}

inline float len(vec2 a)
{
    return sqrt(dot(a, a));
}

inline float clamp(float val, float a, float b)
{
    return (val < a) ? a : (val > b ? b : val);
}

}

#endif