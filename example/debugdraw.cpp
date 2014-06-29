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
    Vec2 to_image(Vec2 v, const Draw_State& state)
    {
        Vec2 inv_dim = sub(state.bounds_max, state.bounds_min);
        inv_dim = make_vec2(1.f / inv_dim.x, 1.f / inv_dim.y);
        return mul(mul(sub(v, state.bounds_min), inv_dim), state.image_dimensions);
    }

    Vec2 from_image(Vec2 v, const Draw_State& state)
    {
        Vec2 inv_dim = make_vec2(1.f/state.image_dimensions.x, 1.f/state.image_dimensions.y);
        return add(state.bounds_min, mul(mul(v, inv_dim), sub(state.bounds_max, state.bounds_min)));
    }

    struct Segment
    {
        Vec2 a;
        Vec2 b;
    };

    Segment to_image(Vec2 a, Vec2 b, const Draw_State& state)
    {
        Segment s;
        s.a = to_image(a, state);
        s.b = to_image(b, state);
        Vec2 d = sub(s.b, s.a);
        float l = len(d);
        d = normalized(d);
        s.b = add(s.a, scale(d, (l >= state.agent_radius) ? l - state.agent_radius : 0.f));
        return s;
    }

    void moveTo(Draw_State& state, Vec2 pos)
    {
        Vec2 pos_img = to_image(pos, state);
        nvgMoveTo(state.vg, pos_img.x, pos_img.y);
    }

    Segment moveTo(Draw_State& state, Vec2 vertex, Vec2 side)
    {
        Segment seg = to_image(vertex, side, state);
        nvgMoveTo(state.vg, seg.b.x, seg.b.y);
        return seg;
    }

    void lineTo(Draw_State& state, Vec2 pos)
    {
        Vec2 pos_img = to_image(pos, state);
        nvgLineTo(state.vg, pos_img.x, pos_img.y);
    }

    Segment lineTo(Draw_State& state, Vec2 vertex, Vec2 side)
    {
        Segment seg = to_image(vertex, side, state);
        nvgLineTo(state.vg, seg.b.x, seg.b.y);
        return seg;
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

    void circle_corner(NVGcontext* vg, Vec2 corner, Vec2 a, Vec2 b, const Draw_State& state, int max_steps=100, int step=0)
    {
        if (step == max_steps || len(sub(b, a)) < 8.f)
        {
            nvgLineTo(vg, b.x, b.y);
            return;
        }

        Vec2 n = normalized(sub(scale(add(a, b), 0.5f), corner));
        Vec2 c = add(corner, scale(n, state.agent_radius));
        circle_corner(vg, corner, a, c, state, max_steps, step + 1);
        circle_corner(vg, corner, c, b, state, max_steps, step + 1);
    }

    void connect_sides(NVGcontext* vg, Vec2 curr_pos, Vec2 curr_side, Vec2 prev_pos, Vec2 prev_side, const Draw_State& state)
    {
        Segment prev_seg = to_image(prev_pos, prev_side, state);
        Segment curr_seg = to_image(curr_pos, curr_side, state);

        if (equal(prev_side, curr_side, 1e-6f))
        {
            Vec2 corner = to_image(curr_side, state);
            Vec2 a = prev_seg.b;
            Vec2 b = curr_seg.b;
            circle_corner(vg, corner, a, b, state);
        }
        else
        {
            nvgLineTo(vg, curr_seg.b.x, curr_seg.b.y);
        }
    }

    void walk_side(NVGcontext* vg, Edge* edge, int dir, const Draw_State& state)
    {
        Half_Edge* dir_edge = edge->dir + (dir^0);
        Half_Edge* opp_edge = edge->dir + (dir^1);

        Vec2 u = target(*state.space, opp_edge)->pos;
        Vec2 v = target(*state.space, dir_edge)->pos;

        Vec2 prev_pos = u;
        Vec2 prev_side = opp_edge->sides[0];

        for (Event* e = event(*state.space, dir_edge); e != 0; e = next(*state.space, e, dir))
        {
            connect_sides(vg, e->pos, e->sides[dir^1], prev_pos, prev_side, state);
            prev_pos = e->pos;
            prev_side = e->sides[dir^1];
        }

        connect_sides(vg, v, dir_edge->sides[1], prev_pos, prev_side, state);
    }

    void fill_edge(NVGcontext* vg, Edge* edge, Draw_State& state)
    {
        Half_Edge* e0 = edge->dir + 0;
        Half_Edge* e1 = edge->dir + 1;

        Vec2 u = source(*state.space, edge)->pos;
        Vec2 v = target(*state.space, edge)->pos;

        nvgBeginPath(vg);

        moveTo(state, u, e1->sides[0]);
        walk_side(vg, edge, 0, state);
        lineTo(state, v);
        lineTo(state, v, e0->sides[0]);
        walk_side(vg, edge, 1, state);
        lineTo(state, u);

        nvgClosePath(vg);
        nvgFill(vg);
    }

    void fill_edge_concave(NVGcontext* vg, Edge* edge, int dir, const Draw_State& state)
    {
        Half_Edge* dir_edge = edge->dir + (dir^0);
        Half_Edge* opp_edge = edge->dir + (dir^1);

        Vec2 u = target(*state.space, opp_edge)->pos;
        Vec2 v = target(*state.space, dir_edge)->pos;

        Vec2 pos = u;
        Vec2 side_l = opp_edge->sides[0];
        Vec2 side_r = opp_edge->sides[1];

        if (event(*state.space, opp_edge) != 0)
        {
            pos = event(*state.space, opp_edge)->pos;
            side_l = event(*state.space, opp_edge)->sides[dir^0];
            side_r = event(*state.space, opp_edge)->sides[dir^1];
        }

        Vec2 c;
        {
            Segment seg_l = to_image(pos, side_l, state);
            Segment seg_r = to_image(pos, side_r, state);
            c = add(seg_l.a, add(sub(seg_l.b, seg_l.a), sub(seg_r.b, seg_l.a)));
        }

        nvgBeginPath(vg);

        {
            Segment seg = to_image(u, opp_edge->sides[1], state);
            nvgMoveTo(vg, seg.a.x, seg.a.y);
            nvgLineTo(vg, seg.b.x, seg.b.y);
        }

        {
            Vec2 prev_pos = from_image(c, state);
            Vec2 prev_side = prev_pos;

            for (Event* e = event(*state.space, dir_edge); e != 0; e = next(*state.space, e, dir^0))
            {
                connect_sides(vg, e->pos, e->sides[dir^0], prev_pos, prev_side, state);
                prev_pos = e->pos;
                prev_side = e->sides[dir^0];
            }
        }

        nvgLineTo(vg, c.x, c.y);

        {
            Vec2 prev_pos = pos;
            Vec2 prev_side = side_l;

            for (Event* e = event(*state.space, opp_edge); e != 0; e = next(*state.space, e, dir^1))
            {
                connect_sides(vg, e->pos, e->sides[dir^1], prev_pos, prev_side, state);
                prev_pos = e->pos;
                prev_side = e->sides[dir^1];
            }
        }

        {
            Segment seg = to_image(u, opp_edge->sides[0], state);
            nvgLineTo(vg, seg.b.x, seg.b.y);
        }

        nvgClosePath(vg);
        nvgFill(vg);
    }

    void stroke_edge(NVGcontext* vg, Edge* edge, const Draw_State& state)
    {
        Half_Edge* e0 = edge->dir + 0;
        Half_Edge* e1 = edge->dir + 1;

        Vec2 u = source(*state.space, edge)->pos;
        Vec2 v = target(*state.space, edge)->pos;

        nvgBeginPath(vg);
        Segment seg = to_image(u, e1->sides[0], state);
        nvgMoveTo(vg, seg.b.x, seg.b.y);

        walk_side(vg, edge, 0, state);

        {
            Segment seg = to_image(v, e0->sides[0], state);
            nvgMoveTo(vg, seg.b.x, seg.b.y);
        }

        walk_side(vg, edge, 1, state);

        nvgStroke(vg);
    }

    void stroke_edge_concave(NVGcontext* vg, Edge* edge, int dir, const Draw_State& state)
    {
        Half_Edge* dir_edge = edge->dir + (dir^0);
        Half_Edge* opp_edge = edge->dir + (dir^1);

        Vec2 u = target(*state.space, opp_edge)->pos;
        Vec2 v = target(*state.space, dir_edge)->pos;

        nvgBeginPath(vg);
        {
            Segment seg = to_image(u, opp_edge->sides[0], state);
            nvgMoveTo(vg, seg.b.x, seg.b.y);
        }

        Vec2 prev_pos = u;
        Vec2 prev_side_l = opp_edge->sides[0];
        Vec2 prev_side_r = opp_edge->sides[1];

        for (Event* e = event(*state.space, dir_edge); e != 0; e = next(*state.space, e, dir))
        {
            connect_sides(vg, e->pos, e->sides[dir^1], prev_pos, prev_side_l, state);
            prev_pos = e->pos;
            prev_side_l = e->sides[dir^1];
        }

        nvgStroke(vg);

        nvgBeginPath(vg);
        {
            Segment seg = to_image(u, opp_edge->sides[1], state);
            nvgMoveTo(vg, seg.b.x, seg.b.y);
        }

        prev_pos = u;

        for (Event* e = event(*state.space, dir_edge); e != 0; e = next(*state.space, e, dir))
        {
            connect_sides(vg, e->pos, e->sides[dir], prev_pos, prev_side_r, state);
            prev_pos = e->pos;
            prev_side_r = e->sides[dir];
        }

        nvgStroke(vg);

        {
            Segment seg_l = to_image(prev_pos, prev_side_l, state);
            Segment seg_r = to_image(prev_pos, prev_side_r, state);
            Vec2 c = add(seg_l.a, add(sub(seg_l.b, seg_l.a), sub(seg_r.b, seg_r.a)));

            nvgBeginPath(vg);
            nvgMoveTo(vg, seg_l.b.x, seg_l.b.y);
            nvgLineTo(vg, c.x, c.y);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, seg_r.b.x, seg_r.b.y);
            nvgLineTo(vg, c.x, c.y);
            nvgStroke(vg);
        }
    }

    void fill_edges(NVGcontext* vg, Draw_State& state)
    {
        NVG_State_Scope s(vg);
        nvgFillColor(vg, nvgRGB(0xff, 0x57, 0x22));

        for (Edge* edge = first(state.space->edges); edge != 0; edge = next(state.space->edges, edge))
        {
            if (degree(*state.space, source(*state.space, edge)) == 1)
            {
                fill_edge_concave(vg, edge, 1, state);
                continue;
            }

            if (degree(*state.space, target(*state.space, edge)) == 1)
            {
                fill_edge_concave(vg, edge, 0, state);
                continue;
            }

            fill_edge(vg, edge, state);
        }

        for (Vertex* vertex = first(state.space->vertices); vertex != 0; vertex = next(state.space->vertices, vertex))
        {
            if (degree(*state.space, vertex) == 2)
            {
                Half_Edge* outgoing_1 = half_edge(*state.space, vertex);
                Half_Edge* outgoing_2 = next(*state.space, outgoing_1);
                Half_Edge* incoming_1 = opposite(*state.space, outgoing_1);
                Half_Edge* incoming_2 = opposite(*state.space, outgoing_2);

                Vec2 o = to_image(vertex->pos, state);

                Segment s10 = to_image(vertex->pos, incoming_1->sides[0], state);
                Segment s21 = to_image(vertex->pos, incoming_2->sides[1], state);
                Vec2 c1 = add(o, add(sub(s10.b, o), sub(s21.b, o)));

                Segment s20 = to_image(vertex->pos, incoming_2->sides[0], state);
                Segment s11 = to_image(vertex->pos, incoming_1->sides[1], state);
                Vec2 c2 = add(o, add(sub(s20.b, o), sub(s11.b, o)));

                if (!equal(s10.b, s21.b, 0.1f))
                {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, s10.b.x, s10.b.y);
                    nvgLineTo(vg, c1.x, c1.y);
                    nvgLineTo(vg, s21.b.x, s21.b.y);
                    nvgLineTo(vg, o.x, o.y);
                    nvgClosePath(vg);
                    nvgFill(vg);
                }

                if (!equal(s20.b, s11.b, 0.1f))
                {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, s20.b.x, s20.b.y);
                    nvgLineTo(vg, c2.x, c2.y);
                    nvgLineTo(vg, s11.b.x, s11.b.y);
                    nvgLineTo(vg, o.x, o.y);
                    nvgClosePath(vg);
                    nvgFill(vg);
                }
            }
        }
    }

    void stroke_borders(NVGcontext* vg, const Draw_State& state)
    {
        NVG_State_Scope s(vg);
        nvgStrokeColor(vg, nvgRGB(120, 0, 0));
        nvgStrokeWidth(vg, 4.f);

        for (Edge* edge = first(state.space->edges); edge != 0; edge = next(state.space->edges, edge))
        {
            if (degree(*state.space, source(*state.space, edge)) == 1)
            {
                stroke_edge_concave(vg, edge, 1, state);
                continue;
            }

            if (degree(*state.space, target(*state.space, edge)) == 1)
            {
                stroke_edge_concave(vg, edge, 0, state);
                continue;
            }

            stroke_edge(vg, edge, state);
        }

        for (Vertex* vertex = first(state.space->vertices); vertex != 0; vertex = next(state.space->vertices, vertex))
        {
            if (degree(*state.space, vertex) == 2)
            {
                Half_Edge* outgoing_1 = half_edge(*state.space, vertex);
                Half_Edge* outgoing_2 = next(*state.space, outgoing_1);
                Half_Edge* incoming_1 = opposite(*state.space, outgoing_1);
                Half_Edge* incoming_2 = opposite(*state.space, outgoing_2);

                Vec2 o = to_image(vertex->pos, state);

                Segment s10 = to_image(vertex->pos, incoming_1->sides[0], state);
                Segment s21 = to_image(vertex->pos, incoming_2->sides[1], state);
                Vec2 c1 = add(o, add(sub(s10.b, o), sub(s21.b, o)));

                Segment s20 = to_image(vertex->pos, incoming_2->sides[0], state);
                Segment s11 = to_image(vertex->pos, incoming_1->sides[1], state);
                Vec2 c2 = add(o, add(sub(s20.b, o), sub(s11.b, o)));

                if (!equal(s10.b, s21.b, 0.1f))
                {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, s10.b.x, s10.b.y);
                    nvgLineTo(vg, c1.x, c1.y);
                    nvgLineTo(vg, s21.b.x, s21.b.y);
                    nvgStroke(vg);
                }

                if (!equal(s20.b, s11.b, 0.1f))
                {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, s20.b.x, s20.b.y);
                    nvgLineTo(vg, c2.x, c2.y);
                    nvgLineTo(vg, s11.b.x, s11.b.y);
                    nvgStroke(vg);
                }
            }
        }
    }

    void draw_background(Draw_State& state)
    {
        NVG_State_Scope s(state.vg);
        nvgFillColor(state.vg, nvgRGB(0x79, 0x55, 0x48));
        nvgBeginPath(state.vg);
        nvgRect(state.vg, 0, 0, state.image_dimensions.x, state.image_dimensions.y);
        nvgFill(state.vg);
    }

    void draw_obstacles(Draw_State& state)
    {
        NVG_State_Scope s(state.vg);
        nvgFillColor(state.vg, nvgRGB(0x18, 0xff, 0xff));

        int offset = 0;
        for (int i = 0; i < state.obstacles->num_polys; ++i)
        {
            nvgBeginPath(state.vg);

            moveTo(state, make_vec2(state.obstacles->x[offset], state.obstacles->y[offset]));

            for (int v = 1; v < state.obstacles->num_poly_verts[i]; ++v)
            {
                lineTo(state, make_vec2(state.obstacles->x[offset + v], state.obstacles->y[offset + v]));
            }

            nvgClosePath(state.vg);
            nvgFill(state.vg);

            offset += state.obstacles->num_poly_verts[i];
        }
    }

    void draw_edges(NVGcontext* vg, const Draw_State& state)
    {
        NVG_State_Scope s(vg);
        nvgStrokeColor(vg, nvgRGB(0xff, 0xeb, 0x3b));
        nvgStrokeWidth(vg, 2.5f);

        for (Edge* edge = first(state.space->edges); edge != 0; edge = next(state.space->edges, edge))
        {
            Half_Edge* e0 = edge->dir + 0;

            Vec2 u = to_image(source(*state.space, e0)->pos, state);
            Vec2 v = to_image(target(*state.space, e0)->pos, state);

            nvgBeginPath(vg);
            nvgMoveTo(vg, u.x, u.y);

            for (Event* e = event(*state.space, e0); e != 0; e = next(*state.space, e, 0))
            {
                Vec2 p = to_image(e->pos, state);
                nvgLineTo(vg, p.x, p.y);
            }

            nvgLineTo(vg, v.x, v.y);
            nvgStroke(vg);
        }
    }

    void draw_events(NVGcontext* vg, const Draw_State& state)
    {
        NVG_State_Scope s(vg);
        nvgBeginPath(vg);
        nvgStrokeColor(vg, nvgRGB(0xff, 0xeb, 0x3b));
        nvgStrokeWidth(vg, 2.5f);

        for (Event* e = first(state.space->events); e != 0; e = next(state.space->events, e))
        {
            Vec2 p = to_image(e->pos, state);
            nvgCircle(vg, p.x, p.y, 4.f);
        }

        nvgStroke(vg);
    }

    void draw_vertices(NVGcontext* vg, const Draw_State& state)
    {
        NVG_State_Scope s(vg);
        nvgBeginPath(vg);
        nvgStrokeColor(vg, nvgRGB(0xff, 0xeb, 0x3b));
        nvgStrokeWidth(vg, 2.5f);

        for (Vertex* v = first(state.space->vertices); v != 0; v = next(state.space->vertices, v))
        {
            Vec2 p = to_image(v->pos, state);
            nvgCircle(vg, p.x, p.y, 8.f);
        }

        nvgStroke(vg);
    }

    void draw_sides(NVGcontext* vg, const Draw_State& state)
    {
        NVG_State_Scope s(vg);
        nvgStrokeColor(vg, nvgRGB(255, 0, 0));
        nvgStrokeWidth(vg, 1.0f);

        for (Edge* edge = first(state.space->edges); edge != 0; edge = next(state.space->edges, edge))
        {
            Half_Edge* e0 = edge->dir + 0;
            Half_Edge* e1 = edge->dir + 1;

            nvgStrokeColor(vg, nvgRGB(255, 0, 0));
            for (Event* e = event(*state.space, e0); e != 0; e = next(*state.space, e, 0))
            {
                Segment s = to_image(e->pos, e->sides[0], state);
                nvgBeginPath(vg);
                nvgMoveTo(vg, s.a.x, s.a.y);
                nvgLineTo(vg, s.b.x, s.b.y);
                nvgStroke(vg);
            }

            nvgStrokeColor(vg, nvgRGB(0, 255, 0));
            for (Event* e = event(*state.space, e0); e != 0; e = next(*state.space, e, 0))
            {
                Segment s = to_image(e->pos, e->sides[1], state);
                nvgBeginPath(vg);
                nvgMoveTo(vg, s.a.x, s.a.y);
                nvgLineTo(vg, s.b.x, s.b.y);
                nvgStroke(vg);
            }
        }
    }
}

void draw_walkable_space(Draw_State& state)
{
    nvgLineCap(state.vg, NVG_ROUND);
    draw_background(state);
    draw_obstacles(state);
    fill_edges(state.vg, state);
    draw_sides(state.vg, state);
    stroke_borders(state.vg, state);
    draw_edges(state.vg, state);
    draw_events(state.vg, state);
    draw_vertices(state.vg, state);
}

}
