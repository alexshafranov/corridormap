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
}

void draw_voronoi_diagram(NVGcontext* vg, const Draw_Params& params)
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

    // edges.
    {
        NVG_State_Scope s(vg);
        nvgStrokeColor(vg, nvgRGB(0, 0, 255));
        nvgStrokeWidth(vg, 2.f);

        for (Edge* edge = first(params.diagram->edges); edge != 0; edge = next(params.diagram->edges, edge))
        {
            Half_Edge* e0 = edge->dir + 0;

            Vec2 u = to_image(source(*params.diagram, e0)->pos, params);
            Vec2 v = to_image(target(*params.diagram, e0)->pos, params);

            nvgBeginPath(vg);
            nvgMoveTo(vg, u.x, u.y);

            for (Event* e = event(*params.diagram, e0); e != 0; e = next(*params.diagram, e, 0))
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

        for (Event* e = first(params.diagram->events); e != 0; e = next(params.diagram->events, e))
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

        for (Vertex* v = first(params.diagram->vertices); v != 0; v = next(params.diagram->vertices, v))
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

        for (Edge* edge = first(params.diagram->edges); edge != 0; edge = next(params.diagram->edges, edge))
        {
            Half_Edge* e0 = edge->dir + 0;
            Vec2 u = to_image(source(*params.diagram, e0)->pos, params);

            nvgBeginPath(vg);
            nvgMoveTo(vg, u.x, u.y);

            for (Event* e = event(*params.diagram, e0); e != 0; e = next(*params.diagram, e, 0))
            {
                Vec2 diff = sub(e->sides[0], e->pos);
                float dist = len(diff);
                Vec2 n = normalized(diff);
                Vec2 p = add(e->pos, scale(n, dist * 0.6f));
                p = to_image(p, params);
                nvgLineTo(vg, p.x, p.y);
            }

            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, u.x, u.y);

            for (Event* e = event(*params.diagram, e0); e != 0; e = next(*params.diagram, e, 1))
            {
                Vec2 diff = sub(e->sides[1], e->pos);
                float dist = len(diff);
                Vec2 n = normalized(diff);
                Vec2 p = add(e->pos, scale(n, dist * 0.6f));
                p = to_image(p, params);
                nvgLineTo(vg, p.x, p.y);
            }

            nvgStroke(vg);
        }
    }
}

}
