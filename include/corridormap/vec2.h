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

#ifndef CORRIDORMAP_VEC2_H_
#define CORRIDORMAP_VEC2_H_

#include <math.h>
#include "corridormap/runtime_types.h"

namespace corridormap {

inline Vec2 make_vec2(float x, float y)
{
    Vec2 result = { x, y };
    return result;
}

inline Vec2 make_vec2(const float* v)
{
    Vec2 result = { v[0], v[1] };
    return result;
}

inline Vec2 operator+(Vec2 a, Vec2 b)
{
    return make_vec2(a.x + b.x, a.y + b.y);
}

inline Vec2 operator-(Vec2 a, Vec2 b)
{
    return make_vec2(a.x - b.x, a.y - b.y);
}

inline Vec2 operator*(Vec2 a, float f)
{
    return make_vec2(a.x*f, a.y*f);
}

inline Vec2 operator*(float f, Vec2 a)
{
    return make_vec2(a.x*f, a.y*f);
}

inline Vec2 operator/(Vec2 a, float f)
{
    return make_vec2(a.x/f, a.y/f);
}

inline Vec2 mul(Vec2 a, Vec2 b)
{
    return make_vec2(a.x*b.x, a.y*b.y);
}

inline float sq(float v)
{
    return v*v;
}

inline Vec2 normalized(Vec2 a)
{
    return a * (1.f/::sqrtf(sq(a.x) + sq(a.y)));
}

inline float dot(Vec2 a, Vec2 b)
{
    return a.x*b.x + a.y*b.y;
}

inline float mag(Vec2 a)
{
    return ::sqrtf(dot(a, a));
}

inline float mag_sq(Vec2 a)
{
    return dot(a, a);
}

inline float det(Vec2 u, Vec2 v)
{
    return u.x*v.y - u.y*v.x;
}

inline float orient(Vec2 o, Vec2 a, Vec2 b)
{
    return det(a - o, b - o);
}

inline bool equal(Vec2 a, Vec2 b, float epsilon)
{
    return sq(a.x - b.x) + sq(a.y - b.y) < sq(epsilon);
}

inline float clamp(float val, float a, float b)
{
    return (val < a) ? a : (val > b ? b : val);
}

}

#endif
