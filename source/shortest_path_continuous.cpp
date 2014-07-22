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

#include <math.h>
#include "corridormap/assert.h"
#include "corridormap/memory.h"
#include "corridormap/vec2.h"
#include "corridormap/runtime.h"

namespace corridormap {

namespace
{
    static const bool direction_incoming = true;
    static const bool direction_outgoing = false;
    static const bool winding_ccw = true;
    static const bool winding_cw  = false;

    Path_Element make_segment(Vec2 p0, Vec2 p1)
    {
        Path_Element result;
        result.type = curve_line;
        result.origin = p0;
        result.p_0 = p0;
        result.p_1 = p1;
        return result;
    }

    Path_Element make_arc(Vec2 origin, Vec2 p0, Vec2 p1, bool ccw)
    {
        Path_Element result;
        result.type = (ccw ? 0x80 : 0x00) | curve_convex_arc;
        result.origin = origin;
        result.p_0 = p0;
        result.p_1 = p1;
        return result;
    }

    // computes tangent point on disk for tangent line going through 'point' towards the disk ['origin', 'radius'].
    // 'ccw' is arc winding (if ccw=true -> circle should be to the left of tangent line).
    // 'incoming' specifies direction of tangent for side test.
    Vec2 get_tangent(Vec2 point, Vec2 origin, float radius, bool ccw, bool incoming)
    {
        Vec2 d = point - origin;
        float l = mag(d);
        corridormap_assert(l >= radius);
        d = d/l;
        float t = sqrtf(sq(l) - sq(radius));

        float sina = t / l;
        float cosa = radius / l;
        Vec2 m11 = make_vec2( cosa, -sina);
        Vec2 m12 = make_vec2( sina,  cosa);
        Vec2 m21 = make_vec2( cosa,  sina);
        Vec2 m22 = make_vec2(-sina,  cosa);

        Vec2 t1 = origin + radius*make_vec2(dot(d, m11), dot(d, m12));
        Vec2 t2 = origin + radius*make_vec2(dot(d, m21), dot(d, m22));

        Vec2 td = t1 - point;
        Vec2 od = origin - point;

        float area = incoming ? det(td, od) : -det(td, od);
        Vec2 t_ccw = area > 0.f ? t1 : t2;
        Vec2 t_cw  = area > 0.f ? t2 : t1;
        return ccw ? t_ccw : t_cw;
    }

    // computes endpoint of the 2*radius sized tangent segment to the point on circle.
    Vec2 get_tangent_at_point(Vec2 point, Vec2 origin, float radius, bool ccw)
    {
        Vec2 d = point - origin;
        corridormap_assert(fabs(mag(d) - radius) < 1e-3f);
        (void)(radius);
        Vec2 e1 = point + make_vec2(-d.y,  d.x);
        Vec2 e2 = point + make_vec2( d.y, -d.x);
        float area = orient(point, e1, origin);
        Vec2 e_ccw = area > 0.f ? e1 : e2;
        Vec2 e_cw  = area > 0.f ? e2 : e1;
        return ccw ? e_ccw : e_cw;
    }

    // computes the mutual tangent segment for two circles (['o1', 'radius'], ['o2', 'radius']).
    // direction of tangent is outgoing from circle 'o1' and incoming to circle 'o2'.
    // that constraint along with the winding of circles allows to select one out of four possible tangents.
    void get_mutual_tangent(Vec2 o1, Vec2 o2, float radius, bool ccw1, bool ccw2, Vec2& t1, Vec2& t2)
    {
        if (ccw1 && ccw2)
        {
            Vec2 d = o2 - o1;
            float s = mag(d);
            corridormap_assert(s > 0.f);
            Vec2 p = make_vec2(-d.y, d.x)*radius/s;
            t1 = o1 - p;
            t2 = o2 - p;
        }
        else if (!ccw1 && !ccw2)
        {
            Vec2 d = o2 - o1;
            float s = mag(d);
            corridormap_assert(s > 0.f);
            Vec2 p = make_vec2(-d.y, d.x)*radius/s;
            t1 = o1 + p;
            t2 = o2 + p;
        }
        else if (ccw1 && !ccw2)
        {
            Vec2 t = get_tangent(o2, o1, 2.f*radius, ccw1, direction_outgoing);
            Vec2 v = (o1 - t)*0.5f;
            t1 = t + v;
            t2 = o2 + v;
        }
        else // if (!ccw1 && ccw2)
        {
            Vec2 t = get_tangent(o1, o2, 2.f*radius, ccw2, direction_incoming);
            Vec2 v = (o2 - t)*0.5f;
            t2 = t + v;
            t1 = o1 + v;
        }
    }

    // returns true if (o, a) -> (o, b) rotation is counter-clockwise.
    bool is_ccw(Vec2 o, Vec2 a, Vec2 b)
    {
        float area = orient(o, a, b);
        return area >= 0.f;
    }

    // checks that point 'p' is inside arc defined by ['a', 'b', 'ccw'].
    bool in_arc(Vec2 p, Vec2 a, Vec2 b, bool ccw)
    {
        float area = orient(a, p, b);
        return ccw ? (area >= -1e-3f) : (area <= 1e-3f);
    }

    // checks that direction (point - start_arc) wraps around arc (i.e. tangent point will be past the arc_start).
    bool wraps_arc(Vec2 origin, float radius, Vec2 arc_start, bool ccw, Vec2 point)
    {
        Vec2 start_tangent = get_tangent_at_point(arc_start, origin, radius, ccw);
        float area = orient(arc_start, start_tangent, point);
        return ccw ? area > 0.f : area < 0.f;
    }

    // internal struct to keep output path state.
    struct Path
    {
        int max_elems;
        int num_elems;
        Path_Element* elems;
    };

    bool full(const Path& p)
    {
        return p.num_elems >= p.max_elems;
    }

    // append a new element to the path.
    // if the new element and the top-most element are both arcs try to merge them.
    // if the new element is close to the top-most, extend the top-most.
    void grow_path(Path& path, const Path_Element& new_element, float epsilon)
    {
        if (path.num_elems > 0)
        {
            Path_Element& previous = path.elems[path.num_elems-1];
            previous.p_1 = new_element.p_0;

            // merge consecutive path arcs.
            if (type(previous) == curve_convex_arc && type(new_element) == curve_convex_arc)
            {
                if (equal(previous.origin, new_element.origin, epsilon))
                {
                    previous.p_1 = new_element.p_1;
                    return;
                }
            }

            // if arc is closed (i.e. extending path with line) remove it if it's too small.
            if (type(previous) == curve_convex_arc && type(new_element) == curve_line)
            {
                if (equal(previous.p_0, previous.p_1, epsilon))
                {
                    previous.type = new_element.type;
                    previous.p_1 = new_element.p_1;
                    return;
                }
            }
        }

        if (path.num_elems < path.max_elems)
        {
            path.elems[path.num_elems++] = new_element;
        }
    }

    // try to add a new element while maintaining the specified funnel side winding.
    void grow_funnel_side(Ring_Buffer<Path_Element>& side, bool ccw, bool& following_border, Path_Element& new_element, float epsilon, float clearance)
    {
        unsigned char curve = type(new_element);
        corridormap_assert(!equal(new_element.p_0, new_element.p_1, epsilon));

        // treat reflex arcs as their chords.
        if (curve == curve_reflex_arc)
        {
            curve = curve_line;
            // opposite winding arcs are never part of a funnel side -> breaks following the border.
            following_border = false;
        }

        // early out: funnel side follows the border.
        if (following_border)
        {
            push_back(side, new_element);
            return;
        }

        // pop segments until empty or the winding invariant is restored.
        while (size(side) > 0)
        {
            Path_Element elem = back(side);

            // adding a vertex.
            if (curve == curve_line)
            {
                // on top of segment.
                if (type(elem) == curve_line)
                {
                    float area = orient(elem.p_0, elem.p_1, new_element.p_1);
                    if (ccw ? area >= 0.f : area <= 0.f)
                    {
                        push_back(side, make_segment(elem.p_1, new_element.p_1));
                        break;
                    }

                    pop_back(side);
                }

                // on top of arc.
                if (type(elem) == curve_convex_arc)
                {
                    Vec2 tangent = get_tangent(new_element.p_1, elem.origin, clearance, is_ccw(elem), direction_outgoing);

                    if (in_arc(tangent, elem.p_0, elem.p_1, is_ccw(elem)))
                    {
                        pop_back(side);
                        push_back(side, make_arc(elem.origin, elem.p_0, tangent, is_ccw(elem)));
                        push_back(side, make_segment(tangent, new_element.p_1));
                        break;
                    }

                    pop_back(side);
                }
            }

            // adding arc.
            if (curve == curve_convex_arc)
            {
                // on top of segment.
                if (type(elem) == curve_line)
                {
                    Vec2 p = elem.p_1;
                    if (equal(p, new_element.p_0, epsilon))
                    {
                        p = elem.p_0;
                    }

                    Vec2 tangent = get_tangent(p, new_element.origin, clearance, ccw, direction_incoming);
                    float area = orient(elem.p_0, elem.p_1, tangent);

                    if (ccw ? area >= 0.f : area <= 0.f)
                    {
                        if (in_arc(tangent, new_element.p_0, new_element.p_1, ccw))
                        {
                            pop_back(side);
                            push_back(side, make_segment(elem.p_0, tangent));
                            push_back(side, make_arc(new_element.origin, tangent, new_element.p_1, ccw));
                            // arcs return funnel side to the border.
                            following_border = true;
                            break;
                        }
                    }

                    pop_back(side);
                }

                // on top of arc.
                if (type(elem) == curve_convex_arc)
                {
                    Vec2 t1;
                    Vec2 t2;
                    get_mutual_tangent(elem.origin, new_element.origin, clearance, ccw, ccw, t1, t2);

                    if (in_arc(t1, elem.p_0, elem.p_1, ccw))
                    {
                        if (in_arc(t2, new_element.p_0, new_element.p_1, ccw))
                        {
                            pop_back(side);
                            push_back(side, make_arc(elem.origin, elem.p_0, t1, ccw));
                            push_back(side, make_segment(t1, t2));
                            push_back(side, make_arc(new_element.origin, t2, new_element.p_1, ccw));
                            // arcs return funnel side to the border.
                            following_border = true;
                            break;
                        }
                    }

                    pop_back(side);
                }
            }
        }
    }

    // moves apex over top-most arc. "eats" the whole arc (if tangent past the end), or cuts it at the tangent point.
    bool move_apex_over_arc(Ring_Buffer<Path_Element>& side, Vec2 tangent, Vec2& apex, Path& path_state, float epsilon)
    {
        Path_Element& arc = front(side);

        // tangent point splits the arc -> grow path by the arc part before tangent point and signal that we're done.
        if (in_arc(tangent, arc.p_0, arc.p_1, is_ccw(arc)))
        {
            grow_path(path_state, make_arc(arc.origin, arc.p_0, tangent, is_ccw(arc)), epsilon);
            apex = tangent;
            arc.p_0 = tangent;
            return true;
        }

        // otherwise grow path by the full arc and continue moving apex.
        grow_path(path_state, pop_front(side), epsilon);
        apex = arc.p_1;
        return false;
    }

    // if failed to add a new element to it's own side (i.e. never satisfied that side winding invariant) then move funnel apex up over the opposite side & grow path.
    void move_funnel_apex(Ring_Buffer<Path_Element>& side, bool ccw, Vec2& apex, const Path_Element& new_element, Path& path, float clearance, float epsilon)
    {
        Vec2 vertex = new_element.p_1;
        Vec2 origin = new_element.origin;
        unsigned char curve = type(new_element);

        // treat reflex arcs as their chords.
        if (curve == curve_reflex_arc)
        {
            curve = curve_line;
        }

        while (size(side) > 0)
        {
            const Path_Element& elem = front(side);

            // adding a vertex.
            if (curve == curve_line)
            {
                // on top of segment.
                if (type(elem) == curve_line)
                {
                    if (is_ccw(elem.p_0, elem.p_1, vertex) != ccw)
                    {
                        break;
                    }

                    apex = elem.p_1;
                    grow_path(path, pop_front(side), epsilon);
                }

                // on top of arc.
                if (type(elem) == curve_convex_arc)
                {
                    if (!wraps_arc(elem.origin, clearance, elem.p_0, is_ccw(elem), vertex))
                    {
                        break;
                    }

                    Vec2 t = get_tangent(vertex, elem.origin, clearance, is_ccw(elem), direction_outgoing);

                    if (move_apex_over_arc(side, t, apex, path, epsilon))
                    {
                        break;
                    }
                }
            }

            // adding an arc.
            if (curve == curve_convex_arc)
            {
                // on top of segment.
                if (type(elem) == curve_line)
                {
                    Vec2 t = get_tangent(elem.p_1, origin, clearance, !ccw, direction_incoming);

                    if (is_ccw(elem.p_0, elem.p_1, t) != ccw)
                    {
                        break;
                    }

                    apex = elem.p_1;
                    grow_path(path, pop_front(side), epsilon);
                }

                // on top of arc.
                if (type(elem) == curve_convex_arc)
                {
                    Vec2 t1;
                    Vec2 t2;
                    get_mutual_tangent(elem.origin, origin, clearance, is_ccw(elem), !ccw, t1, t2);

                    if (!wraps_arc(elem.origin, clearance, elem.p_0, is_ccw(elem), t2))
                    {
                        break;
                    }

                    if (move_apex_over_arc(side, t1, apex, path, epsilon))
                    {
                        break;
                    }
                }
            }
        }
    }

    // try to append a new current element funnel apex was moved over opposite side.
    void restart_funnel_side(Ring_Buffer<Path_Element>& side, bool ccw, Vec2 apex, bool& following_border, const Path_Element& new_element, float clearance)
    {
        unsigned char curve = type(new_element);

        if (curve == curve_reflex_arc)
        {
            curve = curve_line;
        }

        if (curve == curve_line)
        {
            push_back(side, make_segment(apex, new_element.p_1));
        }
        else
        {
            Vec2 tangent = get_tangent(apex, new_element.origin, clearance, ccw, direction_incoming);
            if (in_arc(tangent, new_element.p_0, new_element.p_1, ccw))
            {
                push_back(side, make_segment(apex, tangent));
                push_back(side, make_arc(new_element.origin, tangent, new_element.p_1, ccw));
                following_border = true;
            }
            else
            {
                push_back(side, make_segment(apex, new_element.p_1));
                following_border = false;
            }
        }
    }
}

int find_shortest_path(const Corridor& corridor, Memory* scratch, Vec2 source, Vec2 target, Path_Element* path, int max_path_size)
{
    corridormap_assert(corridor.num_disks > 0);
    Ring_Buffer<Path_Element> funnel_l(scratch, corridor.num_disks);
    Ring_Buffer<Path_Element> funnel_r(scratch, corridor.num_disks);
    corridormap_assert(funnel_l.data != 0);
    corridormap_assert(funnel_r.data != 0);
    Vec2 funnel_apex = source;
    // true if left side topmost element is part of the corridor border.
    bool following_border_l = false;
    // true if right side topmost element is part of the corridor border.
    bool following_border_r = false;

    Path path_state;
    path_state.max_elems = max_path_size;
    path_state.num_elems = 0;
    path_state.elems = path;

    // initialize the funnel.
    push_back(funnel_l, make_segment(funnel_apex, corridor.border_l[0]));
    push_back(funnel_r, make_segment(funnel_apex, corridor.border_r[0]));

    Path_Element elem_l;
    Path_Element elem_r;
    elem_l.p_0 = corridor.border_l[0];
    elem_r.p_0 = corridor.border_r[0];

    for (int i = 1; i < corridor.num_disks; ++i)
    {
        elem_l.p_1 = target;
        elem_l.origin = target;
        elem_l.type = 0x80 | curve_line;

        elem_r.p_1 = target;
        elem_r.origin = target;
        elem_r.type = 0x00 | curve_line;

        if (i < corridor.num_disks-1)
        {
            elem_l.p_1 = corridor.border_l[i];
            elem_r.p_1 = corridor.border_r[i];
            elem_l.origin = corridor.obstacle_l[i];
            elem_r.origin = corridor.obstacle_r[i];
            elem_l.type = 0x80 | (unsigned char)left_border_curve(corridor, i);
            elem_r.type = 0x00 | (unsigned char)right_border_curve(corridor, i);
            corridormap_assert(elem_l.type != curve_point || equal(elem_l.p_0, elem_l.p_1, corridor.epsilon));
            corridormap_assert(elem_r.type != curve_point || equal(elem_r.p_0, elem_r.p_1, corridor.epsilon));
        }

        // add left portal element.
        if (type(elem_l) != curve_point)
        {
            grow_funnel_side(funnel_l, winding_ccw, following_border_l, elem_l, corridor.epsilon, corridor.clearance);
            if (size(funnel_l) == 0)
            {
                move_funnel_apex(funnel_r, winding_cw, funnel_apex, elem_l, path_state, corridor.clearance, corridor.epsilon);
                if (full(path_state)) { return path_state.num_elems; }
                restart_funnel_side(funnel_l, winding_ccw, funnel_apex, following_border_l, elem_l, corridor.clearance);
            }
        }

        // add right portal element.
        if (type(elem_r) != curve_point)
        {
            grow_funnel_side(funnel_r, winding_cw, following_border_r, elem_r, corridor.epsilon, corridor.clearance);
            if (size(funnel_r) == 0)
            {
                move_funnel_apex(funnel_l, winding_ccw, funnel_apex, elem_r, path_state, corridor.clearance, corridor.epsilon);
                if (full(path_state)) { return path_state.num_elems; }
                restart_funnel_side(funnel_r, winding_cw, funnel_apex, following_border_r, elem_r, corridor.clearance);
            }
        }

        elem_l.p_0 = elem_l.p_1;
        elem_r.p_0 = elem_r.p_1;
    }

    return path_state.num_elems;
}

}
