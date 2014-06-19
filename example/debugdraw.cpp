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

#include "corridormap/vec2.h"
#include "corridormap/runtime.h"
#include "debugdraw.h"

namespace corridormap {

namespace
{
    Vec2 to_image(Vec2 v, const Draw_Params& params)
    {
        Vec2 inv_dim = sub(params.bounds_max, params.bounds_min);
        inv_dim = make_vec2(1.f / inv_dim.x, 1.f / inv_dim.y);
        return mul(mul(sub(v, params.bounds_min), inv_dim), params.image_dimensions);
    }

    struct NVG_State_Scope
    {
        NVGcontext* _vg;

        NVG_State_Scope(NVGcontext* vg)
            : _vg(vg)
        {
            nvgSave(_vg);
        }

        ~NVG_State_Scope()
        {
            nvgRestore(_vg);
        }
    };

    struct Segment
    {
        Vec2 a;
        Vec2 b;
    };

    Segment to_image(Vec2 a, Vec2 b, const Draw_Params& params)
    {
        Segment s;
        s.a = to_image(a, params);
        s.b = to_image(b, params);
        Vec2 d = sub(s.b, s.a);
        float l = len(d);
        d = normalized(d);
        s.b = add(s.a, scale(d, (l >= params.agent_radius) ? l - params.agent_radius : 0.f));
        return s;
    }

    void circle_corner(NVGcontext* vg, Vec2 corner, Vec2 a, Vec2 b, const Draw_Params& params)
    {
        if (len(sub(b, a)) < 4.f)
        {
            nvgLineTo(vg, b.x, b.y);
            return;
        }

        Vec2 n = normalized(sub(scale(add(a, b), 0.5f), corner));
        Vec2 c = add(corner, scale(n, params.agent_radius));
        circle_corner(vg, corner, a, c, params);
        circle_corner(vg, corner, c, b, params);
    }

    void connect_sides(NVGcontext* vg, Vec2 curr_pos, Vec2 curr_side, Vec2 prev_pos, Vec2 prev_side, const Draw_Params& params)
    {
        Segment prev_seg = to_image(prev_pos, prev_side, params);
        Segment curr_seg = to_image(curr_pos, curr_side, params);

        if (abs(prev_side.x - curr_side.x) < 0.1f && abs(prev_side.y - curr_side.y) < 0.1f)
        {
            Vec2 corner = to_image(curr_side, params);
            Vec2 a = prev_seg.b;
            Vec2 b = curr_seg.b;
            circle_corner(vg, corner, a, b, params);
        }
        else
        {
            nvgLineTo(vg, curr_seg.b.x, curr_seg.b.y);
        }
    }

    void fill_edges(NVGcontext* vg, const Draw_Params& params)
    {
        NVG_State_Scope s(vg);
        nvgFillColor(vg, nvgRGBA(0, 255, 0, 50));
        nvgStrokeColor(vg, nvgRGB(0, 0, 0));
        nvgStrokeWidth(vg, 2.f);

        for (Edge* edge = first(params.space->edges); edge != 0; edge = next(params.space->edges, edge))
        {
            Half_Edge* e0 = edge->dir + 0;
            Half_Edge* e1 = edge->dir + 1;

            nvgBeginPath(vg);
            Segment seg = to_image(target(*params.space, e1)->pos, e1->sides[1], params);
            nvgMoveTo(vg, seg.b.x, seg.b.y);

            Vec2 prev_pos = target(*params.space, e1)->pos;
            Vec2 prev_side = e1->sides[1];
            for (Event* e = event(*params.space, e0); e != 0; e = next(*params.space, e, 0))
            {
                connect_sides(vg, e->pos, e->sides[1], prev_pos, prev_side, params);
                prev_pos = e->pos;
                prev_side = e->sides[1];
            }

            connect_sides(vg, target(*params.space, e0)->pos, e0->sides[1], prev_pos, prev_side, params);

            {
                Vec2 v = to_image(target(*params.space, e0)->pos, params);
                nvgLineTo(vg, v.x, v.y);
                seg = to_image(target(*params.space, e0)->pos, e0->sides[0], params);
                nvgLineTo(vg, seg.b.x, seg.b.y);
            }

            prev_pos = target(*params.space, e0)->pos;
            prev_side = e0->sides[0];
            for (Event* e = event(*params.space, e1); e != 0; e = next(*params.space, e, 1))
            {
                connect_sides(vg, e->pos, e->sides[0], prev_pos, prev_side, params);
                prev_pos = e->pos;
                prev_side = e->sides[0];
            }

            connect_sides(vg, target(*params.space, e1)->pos, e1->sides[0], prev_pos, prev_side, params);

            {
                Vec2 v = to_image(target(*params.space, e1)->pos, params);
                nvgLineTo(vg, v.x, v.y);
                nvgClosePath(vg);
            }

            nvgFill(vg);
            nvgStroke(vg);
        }
    }
}

void draw_walkable_space(NVGcontext* vg, const Draw_Params& params)
{
    // background.
    {
        NVG_State_Scope s(vg);
        nvgFillColor(vg, nvgRGB(120, 120, 120));
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, params.image_dimensions.x, params.image_dimensions.y);
        nvgFill(vg);
    }

    // obstacles.
    {
        NVG_State_Scope s(vg);
        nvgFillColor(vg, nvgRGB(255, 255, 255));

        int offset = 0;
        for (int i = 0; i < params.obstacles->num_polys; ++i)
        {
            nvgBeginPath(vg);

            Vec2 vert = make_vec2(params.obstacles->x[offset], params.obstacles->y[offset]);
            Vec2 img_vert = to_image(vert, params);
            nvgMoveTo(vg, img_vert.x, img_vert.y);

            for (int v = 1; v < params.obstacles->num_poly_verts[i]; ++v)
            {
                Vec2 vert = make_vec2(params.obstacles->x[offset + v], params.obstacles->y[offset + v]);
                Vec2 img_vert = to_image(vert, params);
                nvgLineTo(vg, img_vert.x, img_vert.y);
            }

            nvgClosePath(vg);
            nvgFill(vg);

            offset += params.obstacles->num_poly_verts[i];
        }
    }

    fill_edges(vg, params);

    // edges.
    {
        NVG_State_Scope s(vg);
        nvgStrokeColor(vg, nvgRGB(0, 0, 255));
        nvgStrokeWidth(vg, 2.f);

        for (Edge* edge = first(params.space->edges); edge != 0; edge = next(params.space->edges, edge))
        {
            Half_Edge* e0 = edge->dir + 0;

            Vec2 u = to_image(source(*params.space, e0)->pos, params);
            Vec2 v = to_image(target(*params.space, e0)->pos, params);

            nvgBeginPath(vg);
            nvgMoveTo(vg, u.x, u.y);

            for (Event* e = event(*params.space, e0); e != 0; e = next(*params.space, e, 0))
            {
                Vec2 p = to_image(e->pos, params);
                nvgLineTo(vg, p.x, p.y);
            }

            nvgLineTo(vg, v.x, v.y);
            nvgStroke(vg);
        }
    }

    // events.
    {
        NVG_State_Scope s(vg);
        nvgBeginPath(vg);
        nvgStrokeColor(vg, nvgRGB(0, 0, 255));
        nvgStrokeWidth(vg, 2.f);

        for (Event* e = first(params.space->events); e != 0; e = next(params.space->events, e))
        {
            Vec2 p = to_image(e->pos, params);
            nvgCircle(vg, p.x, p.y, 4.f);
        }

        nvgStroke(vg);
    }

    // vertices.
    {
        NVG_State_Scope s(vg);
        nvgBeginPath(vg);
        nvgStrokeColor(vg, nvgRGB(0, 0, 255));
        nvgStrokeWidth(vg, 2.f);

        for (Vertex* v = first(params.space->vertices); v != 0; v = next(params.space->vertices, v))
        {
            Vec2 p = to_image(v->pos, params);
            nvgCircle(vg, p.x, p.y, 8.f);
        }

        nvgStroke(vg);
    }

    // corridor bounds.
    {
        NVG_State_Scope s(vg);
        nvgStrokeColor(vg, nvgRGB(255, 0, 0));
        nvgStrokeWidth(vg, 2.f);

        for (Edge* edge = first(params.space->edges); edge != 0; edge = next(params.space->edges, edge))
        {
            Half_Edge* e0 = edge->dir + 0;
            Half_Edge* e1 = edge->dir + 1;

            nvgStrokeColor(vg, nvgRGB(255, 0, 0));
            for (Event* e = event(*params.space, e0); e != 0; e = next(*params.space, e, 0))
            {
                Segment s = to_image(e->pos, e->sides[0], params);
                nvgBeginPath(vg);
                nvgMoveTo(vg, s.a.x, s.a.y);
                nvgLineTo(vg, s.b.x, s.b.y);
                nvgStroke(vg);
            }

            nvgStrokeColor(vg, nvgRGB(0, 255, 0));
            for (Event* e = event(*params.space, e0); e != 0; e = next(*params.space, e, 0))
            {
                Segment s = to_image(e->pos, e->sides[1], params);
                nvgBeginPath(vg);
                nvgMoveTo(vg, s.a.x, s.a.y);
                nvgLineTo(vg, s.b.x, s.b.y);
                nvgStroke(vg);
            }

            // vertex closest points.

            nvgStrokeColor(vg, nvgRGB(255, 0, 255));
            {
                nvgBeginPath(vg);
                Segment s = to_image(target(*params.space, e0)->pos, e0->sides[0], params);
                nvgMoveTo(vg, s.a.x, s.a.y);
                nvgLineTo(vg, s.b.x, s.b.y);
                nvgStroke(vg);
            }

            {
                nvgBeginPath(vg);
                Segment s = to_image(target(*params.space, e0)->pos, e0->sides[1], params);
                nvgMoveTo(vg, s.a.x, s.a.y);
                nvgLineTo(vg, s.b.x, s.b.y);
                nvgStroke(vg);
            }

            {
                nvgBeginPath(vg);
                Segment s = to_image(target(*params.space, e1)->pos, e1->sides[0], params);
                nvgMoveTo(vg, s.a.x, s.a.y);
                nvgLineTo(vg, s.b.x, s.b.y);
                nvgStroke(vg);
            }

            {
                nvgBeginPath(vg);
                Segment s = to_image(target(*params.space, e1)->pos, e1->sides[1], params);
                nvgMoveTo(vg, s.a.x, s.a.y);
                nvgLineTo(vg, s.b.x, s.b.y);
                nvgStroke(vg);
            }
        }
    }
}

}
