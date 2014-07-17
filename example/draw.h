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

#ifndef CORRIDORMAP_EXAMPLE_DEBUGDRAW_H_
#define CORRIDORMAP_EXAMPLE_DEBUGDRAW_H_

#include "nanovg.h"
#include "corridormap/build_types.h"
#include "corridormap/runtime_types.h"

namespace corridormap {

struct Draw_State
{
    float agent_radius;
    Vec2 bounds_min;
    Vec2 bounds_max;
    Vec2 image_dimensions;
    Footprint* obstacles;
    Walkable_Space* space;
    NVGcontext* vg;
};

void draw_walkable_space(Draw_State& state);
void draw_corridor(Draw_State& state, Corridor& corridor);
void draw_portals(Draw_State& state, Corridor& corridor);
void draw_path(Draw_State& state, Corridor& corridor, Vec2 source, Vec2 target);

}

#endif
