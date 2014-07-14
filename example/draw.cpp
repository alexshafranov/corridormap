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
#include "draw.h"

namespace corridormap {

namespace
{
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

    struct Border_Line_State
    {
        Vec2 prev_pos;
        Vec2 prev_side;
    };

    Vec2 to_image(Vec2 v, const Draw_State& state)
    {
        Vec2 inv_dim = sub(state.bounds_max, state.bounds_min);
        inv_dim = make_vec2(1.f / inv_dim.x, 1.f / inv_dim.y);
        Vec2 norm_v = mul(sub(v, state.bounds_min), inv_dim);
        norm_v.y = 1.f - norm_v.y;
        return mul(norm_v, state.image_dimensions);
    }

    Vec2 from_image(Vec2 v, const Draw_State& state)
    {
        Vec2 inv_dim = make_vec2(1.f / state.image_dimensions.x, 1.f / state.image_dimensions.y);
        Vec2 norm_v = mul(v, inv_dim);
        norm_v.y = 1.f - norm_v.y;
        return add(state.bounds_min, mul(norm_v, sub(state.bounds_max, state.bounds_min)));
    }

    Segment to_image(Vec2 a, Vec2 b, const Draw_State& state)
    {
        Segment s;
        s.a = to_image(a, state);
        s.b = to_image(b, state);

        Vec2 d = sub(s.b, s.a);
        float l = mag(d);
        if (l > 0.f)
        {
            d = normalized(d);
            s.b = add(s.a, scale(d, (l >= state.agent_radius) ? l - state.agent_radius : 0.f));
        }

        return s;
    }

    void move_to(Draw_State& state, Vec2 pos)
    {
        Vec2 pos_img = to_image(pos, state);
        nvgMoveTo(state.vg, pos_img.x, pos_img.y);
    }

    Segment move_to(Draw_State& state, Vec2 vertex, Vec2 side)
    {
        Segment seg = to_image(vertex, side, state);
        nvgMoveTo(state.vg, seg.b.x, seg.b.y);
        return seg;
    }

    void line_to(Draw_State& state, Vec2 pos)
    {
        Vec2 pos_img = to_image(pos, state);
        nvgLineTo(state.vg, pos_img.x, pos_img.y);
    }

    Segment line_to(Draw_State& state, Vec2 vertex, Vec2 side)
    {
        Segment seg = to_image(vertex, side, state);
        nvgLineTo(state.vg, seg.b.x, seg.b.y);
        return seg;
    }

    void arc(Draw_State& state, Vec2 origin, Vec2 a, Vec2 b, int max_steps=4, int step=0)
    {
        Vec2 a_img = to_image(a, state);
        Vec2 b_img = to_image(b, state);
        Vec2 origin_img = to_image(origin, state);
        float radius = mag(sub(a_img, origin_img));

        if (step == max_steps || mag(sub(b_img, a_img)) < 8.f)
        {
            nvgLineTo(state.vg, b_img.x, b_img.y);
            return;
        }

        Vec2 n = normalized(sub(scale(add(a_img, b_img), 0.5f), origin_img));
        Vec2 c_img = add(origin_img, scale(n, radius));
        Vec2 c = from_image(c_img, state);

        arc(state, origin, a, c, max_steps, step + 1);
        arc(state, origin, c, b, max_steps, step + 1);
    }

    void circle(Draw_State& state, Vec2 origin, float radius)
    {
        Vec2 o = to_image(origin, state);
        nvgCircle(state.vg, o.x, o.y, radius);
    }

    void circle_corner(NVGcontext* vg, Vec2 corner, Vec2 a, Vec2 b, const Draw_State& state, int max_steps=100, int step=0)
    {
        if (step == max_steps || mag(sub(b, a)) < 8.f)
        {
            nvgLineTo(vg, b.x, b.y);
            return;
        }

        Vec2 n = normalized(sub(scale(add(a, b), 0.5f), corner));
        Vec2 c = add(corner, scale(n, state.agent_radius));
        circle_corner(vg, corner, a, c, state, max_steps, step + 1);
        circle_corner(vg, corner, c, b, state, max_steps, step + 1);
    }

    Border_Line_State begin_border(Draw_State& state, Vec2 start_pos, Vec2 start_side, bool start_path=true)
    {
        Border_Line_State border;
        border.prev_pos = start_pos;
        border.prev_side = start_side;

        if (start_path)
        {
            move_to(state, start_pos, start_side);
        }
        else
        {
            line_to(state, start_pos, start_side);
        }

        return border;
    }

    void next_border_point(Draw_State& state, Border_Line_State& border, Vec2 pos, Vec2 side)
    {
        Segment prev_seg = to_image(border.prev_pos, border.prev_side, state);
        Segment curr_seg = to_image(pos, side, state);

        if (equal(border.prev_side, side, 0.05f))
        {
            Vec2 corner = to_image(side, state);
            Vec2 a = prev_seg.b;
            Vec2 b = curr_seg.b;
            circle_corner(state.vg, corner, a, b, state);
        }
        else
        {
            nvgLineTo(state.vg, curr_seg.b.x, curr_seg.b.y);
        }

        border.prev_pos = pos;
        border.prev_side = side;
    }

    void walk_events_right_side(Draw_State& state, Border_Line_State& border, Half_Edge* dir_edge)
    {
        for (Event* e = event(*state.space, dir_edge); e != 0; e = next(*state.space, dir_edge, e))
        {
            next_border_point(state, border, e->pos, right_side(*state.space, dir_edge, e));
        }
    }

    void fill_edge(Draw_State& state, Edge* edge)
    {
        Half_Edge* e0 = edge->dir + 0;
        Half_Edge* e1 = edge->dir + 1;

        Vec2 u = source(*state.space, edge)->pos;
        Vec2 v = target(*state.space, edge)->pos;

        nvgBeginPath(state.vg);

        Border_Line_State right_border = begin_border(state, u, left_side(*state.space, e1), true);
        walk_events_right_side(state, right_border, e0);
        next_border_point(state, right_border, v, right_side(*state.space, e0));
        line_to(state, v);
        Border_Line_State left_border = begin_border(state, v, left_side(*state.space, e0), false);
        walk_events_right_side(state, left_border, e1);
        next_border_point(state, left_border, u, right_side(*state.space, e1));
        line_to(state, u);
        nvgClosePath(state.vg);

        nvgFill(state.vg);
    }

    void stroke_edge(Draw_State& state, Edge* edge)
    {
        Half_Edge* e0 = edge->dir + 0;
        Half_Edge* e1 = edge->dir + 1;

        Vec2 u = source(*state.space, edge)->pos;
        Vec2 v = target(*state.space, edge)->pos;

        nvgBeginPath(state.vg);
        Border_Line_State right_border = begin_border(state, u, left_side(*state.space, e1));
        walk_events_right_side(state, right_border, e0);
        next_border_point(state, right_border, v, right_side(*state.space, e0));
        nvgStroke(state.vg);

        nvgBeginPath(state.vg);
        Border_Line_State left_border = begin_border(state, v, left_side(*state.space, e0));
        walk_events_right_side(state, left_border, e1);
        next_border_point(state, left_border, u, right_side(*state.space, e1));
        nvgStroke(state.vg);
    }

    Vec2 concave_corner(Draw_State& state, Half_Edge* dir_edge)
    {
        Half_Edge* opp_edge = opposite(*state.space, dir_edge);

        Vec2 pos = target(*state.space, opp_edge)->pos;
        Vec2 side_l = left_side(*state.space, opp_edge);
        Vec2 side_r = right_side(*state.space, opp_edge);

        Event* first_event = event(*state.space, opp_edge);
        if (first_event != 0)
        {
            pos = first_event->pos;
            side_l = left_side(*state.space, opp_edge, first_event);
            side_r = right_side(*state.space, opp_edge, first_event);
        }

        Segment seg_l = to_image(pos, side_l, state);
        Segment seg_r = to_image(pos, side_r, state);

        Vec2 v_corner = add(seg_l.a, add(sub(seg_l.b, seg_l.a), sub(seg_r.b, seg_l.a)));
        return from_image(v_corner, state);
    }

    void fill_edge_concave(Draw_State& state, Half_Edge* dir_edge)
    {
        Half_Edge* opp_edge = opposite(*state.space, dir_edge);
        Vec2 corner = concave_corner(state, dir_edge);
        Vec2 u = source(*state.space, dir_edge)->pos;
        Vec2 v = target(*state.space, dir_edge)->pos;

        nvgBeginPath(state.vg);

        Border_Line_State left_border = begin_border(state, u, left_side(*state.space, opp_edge), true);
        walk_events_right_side(state, left_border, dir_edge);
        next_border_point(state, left_border, corner, corner);

        Border_Line_State right_border = begin_border(state, corner, corner, false);
        walk_events_right_side(state, right_border, opp_edge);
        next_border_point(state, right_border, u, right_side(*state.space, opp_edge));
        line_to(state, u);

        nvgClosePath(state.vg);
        nvgFill(state.vg);
    }

    void stroke_edge_concave(Draw_State& state, Half_Edge* dir_edge)
    {
        Half_Edge* opp_edge = opposite(*state.space, dir_edge);
        Vec2 corner = concave_corner(state, dir_edge);
        Vec2 u = source(*state.space, dir_edge)->pos;
        Vec2 v = target(*state.space, dir_edge)->pos;

        nvgBeginPath(state.vg);
        Border_Line_State left_border = begin_border(state, u, left_side(*state.space, opp_edge));
        walk_events_right_side(state, left_border, dir_edge);
        next_border_point(state, left_border, corner, corner);
        nvgStroke(state.vg);

        nvgBeginPath(state.vg);
        Border_Line_State right_border = begin_border(state, corner, corner);
        walk_events_right_side(state, right_border, opp_edge);
        next_border_point(state, right_border, u, right_side(*state.space, opp_edge));
        nvgStroke(state.vg);
    }

    void fill_edges(NVGcontext* vg, Draw_State& state)
    {
        NVG_State_Scope s(vg);
        nvgFillColor(vg, nvgRGB(0xff, 0x57, 0x22));

        for (Edge* edge = first(state.space->edges); edge != 0; edge = next(state.space->edges, edge))
        {
            if (degree(*state.space, source(*state.space, edge)) == 1)
            {
                fill_edge_concave(state, edge->dir + 1);
                continue;
            }

            if (degree(*state.space, target(*state.space, edge)) == 1)
            {
                fill_edge_concave(state, edge->dir + 0);
                continue;
            }

            fill_edge(state, edge);
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

    void stroke_borders(NVGcontext* vg, Draw_State& state)
    {
        NVG_State_Scope s(vg);
        nvgStrokeColor(vg, nvgRGB(120, 0, 0));
        nvgStrokeWidth(vg, 4.f);

        for (Edge* edge = first(state.space->edges); edge != 0; edge = next(state.space->edges, edge))
        {
            if (degree(*state.space, source(*state.space, edge)) == 1)
            {
                stroke_edge_concave(state, edge->dir + 1);
                continue;
            }

            if (degree(*state.space, target(*state.space, edge)) == 1)
            {
                stroke_edge_concave(state, edge->dir + 0);
                continue;
            }

            stroke_edge(state, edge);
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

            move_to(state, make_vec2(state.obstacles->x[offset], state.obstacles->y[offset]));

            for (int v = 1; v < state.obstacles->num_poly_verts[i]; ++v)
            {
                line_to(state, make_vec2(state.obstacles->x[offset + v], state.obstacles->y[offset + v]));
            }

            nvgClosePath(state.vg);
            nvgFill(state.vg);

            offset += state.obstacles->num_poly_verts[i];
        }
    }

    void draw_edges(Draw_State& state)
    {
        NVG_State_Scope s(state.vg);
        nvgStrokeColor(state.vg, nvgRGB(0xff, 0xeb, 0x3b));
        nvgStrokeWidth(state.vg, 2.5f);

        for (Edge* edge = first(state.space->edges); edge != 0; edge = next(state.space->edges, edge))
        {
            Half_Edge* e0 = edge->dir + 0;

            Vec2 u = source(*state.space, e0)->pos;
            Vec2 v = target(*state.space, e0)->pos;

            nvgBeginPath(state.vg);
            move_to(state, u);

            for (Event* e = event(*state.space, e0); e != 0; e = next(*state.space, e0, e))
            {
                line_to(state, e->pos);
            }

            line_to(state, v);
            nvgStroke(state.vg);
        }
    }

    void draw_events(Draw_State& state)
    {
        NVG_State_Scope s(state.vg);
        nvgBeginPath(state.vg);
        nvgStrokeColor(state.vg, nvgRGB(0xff, 0xeb, 0x3b));
        nvgStrokeWidth(state.vg, 2.5f);

        for (Event* e = first(state.space->events); e != 0; e = next(state.space->events, e))
        {
            circle(state, e->pos, 4.f);
        }

        nvgStroke(state.vg);
    }

    void draw_vertices(Draw_State& state)
    {
        NVG_State_Scope s(state.vg);
        nvgStrokeColor(state.vg, nvgRGB(0xff, 0xeb, 0x3b));
        nvgStrokeWidth(state.vg, 2.5f);
        nvgBeginPath(state.vg);

        for (Vertex* v = first(state.space->vertices); v != 0; v = next(state.space->vertices, v))
        {
            circle(state, v->pos, 8.f);
        }

        nvgStroke(state.vg);
    }

    void draw_sides(Draw_State& state)
    {
        NVG_State_Scope s(state.vg);
        nvgStrokeColor(state.vg, nvgRGB(255, 0, 0));
        nvgStrokeWidth(state.vg, 1.0f);

        for (Edge* edge = first(state.space->edges); edge != 0; edge = next(state.space->edges, edge))
        {
            for (Event* e = event(*state.space, edge->dir); e != 0; e = next(*state.space, edge->dir, e))
            {
                nvgStrokeColor(state.vg, nvgRGB(255, 0, 0));
                nvgBeginPath(state.vg);
                move_to(state, e->pos);
                line_to(state, e->pos, left_side(*state.space, edge->dir, e));
                nvgStroke(state.vg);

                nvgStrokeColor(state.vg, nvgRGB(0, 255, 0));
                nvgBeginPath(state.vg);
                move_to(state, e->pos);
                line_to(state, e->pos, right_side(*state.space, edge->dir, e));
                nvgStroke(state.vg);
            }
        }
    }
}

void draw_walkable_space(Draw_State& state)
{
    NVG_State_Scope s(state.vg);
    nvgLineCap(state.vg, NVG_ROUND);

    draw_background(state);
    draw_obstacles(state);
    fill_edges(state.vg, state);
    draw_sides(state);
    stroke_borders(state.vg, state);
    draw_edges(state);
    draw_events(state);
    draw_vertices(state);
}

void draw_corridor(Draw_State& state, Corridor& corridor)
{
    if (corridor.num_disks <= 0)
    {
        return;
    }

    NVG_State_Scope s(state.vg);
    nvgLineCap(state.vg, NVG_ROUND);
    nvgFillColor(state.vg, nvgRGBA(0, 0, 0, 127));
    nvgStrokeColor(state.vg, nvgRGB(255, 255, 255));
    nvgStrokeWidth(state.vg, 2.f);
    nvgBeginPath(state.vg);

    move_to(state, corridor.right_b[0]);

    for (int i = 1; i < corridor.num_disks; ++i)
    {
        Vec2 src = corridor.right_b[i-1];
        Vec2 tgt = corridor.right_b[i];

        if ((corridor.curves[i] & 0xf0) >> 4 == border_type_arc_vertex)
        {
            arc(state, corridor.origins[i], src, tgt);
        }
        else
        {
            line_to(state, tgt);
        }
    }

    line_to(state, corridor.left_b[corridor.num_disks-1]);

    for (int i = corridor.num_disks-1; i > 0; --i)
    {
        Vec2 src = corridor.left_b[i];
        Vec2 tgt = corridor.left_b[i-1];

        if ((corridor.curves[i] & 0x0f) >> 0 == border_type_arc_vertex)
        {
            arc(state, corridor.origins[i], src, tgt);
        }
        else
        {
            line_to(state, tgt);
        }
    }

    line_to(state, corridor.right_b[0]);

    nvgFill(state.vg);
    nvgStroke(state.vg);
}

}
