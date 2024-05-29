// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.

#include "NvVec3.h"
#include "NvVec2.h"
#include "NvBounds3.h"
#include <vector>
#include <queue>
#include <map>

#include <NvBlastExtAuthoringMeshCleanerImpl.h>
#include <NvBlastExtAuthoringMeshImpl.h>
#include <NvBlastExtAuthoringInternalCommon.h>
#include <NvBlastNvSharedHelpers.h>
#include <boost/multiprecision/cpp_int.hpp>

using namespace nvidia;

using namespace Nv::Blast;
using namespace boost::multiprecision;

/**
    Exact rational vector types.
*/
struct RVec3
{
    cpp_rational x, y, z;
    RVec3() {}

    bool isZero()
    {
        return x.is_zero() && y.is_zero() && z.is_zero();
    }

    RVec3(cpp_rational _x, cpp_rational _y, cpp_rational _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    RVec3(const NvcVec3& p)
    {
        x = cpp_rational(p.x);
        y = cpp_rational(p.y);
        z = cpp_rational(p.z);
    }
    NvVec3 toVec3()
    {
        return { x.convert_to<float>(), y.convert_to<float>(), z.convert_to<float>() };
    }

    RVec3 operator-(const RVec3& b) const
    {
        return RVec3(x - b.x, y - b.y, z - b.z);
    }
    RVec3 operator+(const RVec3& b) const
    {
        return RVec3(x + b.x, y + b.y, z + b.z);
    }
    RVec3 cross(const RVec3& in) const
    {
        return RVec3(y * in.z - in.y * z, in.x * z - x * in.z, x * in.y - in.x * y);
    }
    cpp_rational dot(const RVec3& in) const
    {
        return x * in.x + y * in.y + z * in.z;
    }
    RVec3 operator*(const cpp_rational& in) const
    {
        return RVec3(x * in, y * in, z * in);
    }
};

struct RVec2
{
    cpp_rational x, y;
    RVec2() {}

    RVec2(cpp_rational _x, cpp_rational _y)
    {
        x = _x;
        y = _y;
    }

    RVec2(const NvcVec2& p)
    {
        x = cpp_rational(p.x);
        y = cpp_rational(p.y);
    }
    NvVec2 toVec2()
    {
        return { x.convert_to<float>(), y.convert_to<float>() };
    }

    RVec2 operator-(const RVec2& b) const
    {
        return RVec2(x - b.x, y - b.y);
    }
    RVec2 operator+(const RVec2& b) const
    {
        return RVec2(x + b.x, y + b.y);
    }
    cpp_rational cross(const RVec2& in) const
    {
        return x * in.y - y * in.x;
    }
    cpp_rational dot(const RVec2& in) const
    {
        return x * in.x + y * in.y;
    }
    RVec2 operator*(const cpp_rational& in) const
    {
        return RVec2(x * in, y * in);
    }
};
struct RatPlane
{
    RVec3 n;
    cpp_rational d;

    RatPlane(const RVec3& a, const RVec3& b, const RVec3& c)
    {
        n = (b - a).cross(c - a);
        d = -n.dot(a);
    };
    cpp_rational distance(RVec3& in)
    {
        return n.dot(in) + d;
    }
};

bool isSame(const RatPlane& a, const RatPlane& b)
{
    if (a.d != b.d)
        return false;
    if (a.n.x != b.n.x || a.n.y != b.n.y || a.n.z != b.n.z)
        return false;
    return true;
}

RVec3 planeSegmInters(RVec3& a, RVec3& b, RatPlane& pl)
{
    cpp_rational t = -(a.dot(pl.n) + pl.d) / pl.n.dot(b - a);
    RVec3 on       = a + (b - a) * t;
    return on;
}

enum POINT_CLASS
{
    ON_AB = 0,
    ON_BC = 1,
    ON_AC = 2,
    INSIDE_TR,
    OUTSIDE_TR,
    ON_VERTEX
};


int32_t isPointInside(const RVec2& a, const RVec2& b, const RVec2& c, const RVec2& p)
{
    cpp_rational v1 = (b - a).cross(p - a);
    cpp_rational v2 = (c - b).cross(p - b);
    cpp_rational v3 = (a - c).cross(p - c);


    int32_t v1s = v1.sign();
    int32_t v2s = v2.sign();
    int32_t v3s = v3.sign();

    if (v1s * v2s < 0 || v1s * v3s < 0 || v2s * v3s < 0)
        return OUTSIDE_TR;

    if (v1s == 0 && v2s == 0)
        return OUTSIDE_TR;
    if (v1s == 0 && v3s == 0)
        return OUTSIDE_TR;
    if (v2s == 0 && v3s == 0)
        return OUTSIDE_TR;

    if (v1s == 0)
        return ON_AB;
    if (v2s == 0)
        return ON_BC;
    if (v3s == 0)
        return ON_AC;

    return INSIDE_TR;
}

RVec2 getProjectedPointWithWinding(const RVec3& point, ProjectionDirections dir)
{
    if (dir & YZ_PLANE)
    {
        if (dir & OPPOSITE_WINDING)
        {
            return RVec2(point.z, point.y);
        }
        else
            return RVec2(point.y, point.z);
    }
    if (dir & ZX_PLANE)
    {
        if (dir & OPPOSITE_WINDING)
        {
            return RVec2(point.z, point.x);
        }
        return RVec2(point.x, point.z);
    }
    if (dir & OPPOSITE_WINDING)
    {
        return RVec2(point.y, point.x);
    }
    return RVec2(point.x, point.y);
}

struct DelTriangle
{
    int32_t p[3];
    int32_t n[3];
    int32_t parentTriangle;
    int32_t getEdWP(int32_t vrt)
    {
        if (p[0] == vrt)
            return 1;
        if (p[1] == vrt)
            return 2;
        if (p[2] == vrt)
            return 0;
        return -1;
    }
    int32_t getEdId(int32_t v1, int32_t v2)
    {
        if (p[0] == v1 && p[1] == v2)
            return 0;
        if (p[1] == v1 && p[2] == v2)
            return 1;
        if (p[2] == v1 && p[0] == v2)
            return 2;
        return -1;
    }
    int32_t getOppP(int32_t v1, int32_t v2)
    {
        if (p[0] == v1 && p[1] == v2)
            return 2;
        if (p[1] == v1 && p[2] == v2)
            return 0;
        if (p[2] == v1 && p[0] == v2)
            return 1;
        return -1;
    }

    int32_t getOppPoint(int32_t v1, int32_t v2)
    {
        if (p[0] != v1 && p[0] != v2)
            return p[0];
        if (p[1] != v1 && p[1] != v2)
            return p[1];
        if (p[2] != v1 && p[2] != v2)
            return p[2];
        return -1;
    }
    bool compare(const DelTriangle& t) const
    {
        if (p[0] == t.p[0] && p[1] == t.p[1] && p[2] == t.p[2])
            return true;
        if (p[1] == t.p[0] && p[2] == t.p[1] && p[0] == t.p[2])
            return true;
        if (p[2] == t.p[0] && p[0] == t.p[1] && p[1] == t.p[2])
            return true;
        return false;
    }
};

struct DelEdge
{
    int32_t s, e;
    int32_t nr, nl;
};


bool isIntersectsTriangle(RVec2& a, RVec2& b, RVec2& c, RVec2& s, RVec2& e)
{
    RVec2 vec = e - s;

    if ((a - s).cross(vec) * (b - s).cross(vec) < 0)
    {
        RVec2 vec2 = b - a;
        if ((s - a).cross(vec2) * (e - a).cross(vec) < 0)
            return true;
    }

    if ((b - s).cross(vec) * (c - s).cross(vec) < 0)
    {
        RVec2 vec2 = c - b;
        if ((s - b).cross(vec2) * (e - b).cross(vec) < 0)
            return true;
    }

    if ((a - s).cross(vec) * (c - s).cross(vec) < 0)
    {
        RVec2 vec2 = a - c;
        if ((s - c).cross(vec2) * (e - c).cross(vec) < 0)
            return true;
    }

    return false;
}


inline int32_t inCircumcircle(RVec2& a, RVec2& b, RVec2& c, RVec2& p)
{
    RVec2 ta        = a - p;
    RVec2 tb        = b - p;
    RVec2 tc        = c - p;
    cpp_rational ad = ta.dot(ta);
    cpp_rational bd = tb.dot(tb);
    cpp_rational cd = tc.dot(tc);

    cpp_rational pred =
        ta.x * (tb.y * cd - tc.y * bd) - ta.y * (tb.x * cd - tc.x * bd) + ad * (tb.x * tc.y - tc.x * tb.y);


    if (pred > 0)
        return 1;
    if (pred < 0)
        return -1;
    return 0;
}

int32_t getEdge(std::vector<DelEdge>& edges, int32_t s, int32_t e)
{
    for (uint32_t i = 0; i < edges.size(); ++i)
    {
        if (edges[i].s == s && edges[i].e == e)
            return i;
    }

    edges.push_back(DelEdge());
    edges.back().s = s;
    edges.back().e = e;
    return edges.size() - 1;
}

void reubildAdjacency(std::vector<DelTriangle>& state)
{
    for (uint32_t i = 0; i < state.size(); ++i)
    {
        state[i].n[0] = state[i].n[1] = state[i].n[2] = -1;
    }
    for (uint32_t i = 0; i < state.size(); ++i)
    {
        if (state[i].p[0] == -1)
            continue;
        for (uint32_t j = i + 1; j < state.size(); ++j)
        {
            if (state[j].p[0] == -1)
                continue;
            for (uint32_t k = 0; k < 3; ++k)
            {
                for (uint32_t c = 0; c < 3; ++c)
                {
                    if (state[i].p[k] == state[j].p[(c + 1) % 3] && state[i].p[(k + 1) % 3] == state[j].p[c])
                    {
                        state[i].n[k] = j;
                        state[j].n[c] = i;
                    }
                }
            }
        }
    }
}

void insertPoint(std::vector<RVec2>& vertices, std::vector<DelTriangle>& state, int32_t p, const std::vector<Edge>& edges)
{
    std::queue<int32_t> triangleToCheck;

    for (uint32_t i = 0; i < state.size(); ++i)
    {
        if (state[i].p[0] == -1)
            continue;
        DelTriangle ctr = state[i];
        int32_t cv      = isPointInside(vertices[ctr.p[0]], vertices[ctr.p[1]], vertices[ctr.p[2]], vertices[p]);

        if (cv == OUTSIDE_TR)
            continue;
        if (cv == INSIDE_TR)
        {
            uint32_t taInd = state.size();
            uint32_t tbInd = state.size() + 1;
            uint32_t tcInd = state.size() + 2;
            state.resize(state.size() + 3);

            state[taInd].p[0] = ctr.p[2];
            state[taInd].p[1] = ctr.p[0];
            state[taInd].p[2] = p;

            state[taInd].n[0] = ctr.n[2];
            state[taInd].n[1] = tbInd;
            state[taInd].n[2] = tcInd;

            state[tbInd].p[0] = ctr.p[0];
            state[tbInd].p[1] = ctr.p[1];
            state[tbInd].p[2] = p;

            state[tbInd].n[0] = ctr.n[0];
            state[tbInd].n[1] = tcInd;
            state[tbInd].n[2] = taInd;

            state[tcInd].p[0] = ctr.p[1];
            state[tcInd].p[1] = ctr.p[2];
            state[tcInd].p[2] = p;

            state[tcInd].n[0] = ctr.n[1];
            state[tcInd].n[1] = taInd;
            state[tcInd].n[2] = tbInd;


            triangleToCheck.push(taInd);
            triangleToCheck.push(tbInd);
            triangleToCheck.push(tcInd);


            /**
            Change neighbors
            */
            int32_t nb = state[i].n[0];
            if (nb != -1)
                state[nb].n[state[nb].getEdId(state[i].p[1], state[i].p[0])] = tbInd;
            nb = state[i].n[1];
            if (nb != -1)
                state[nb].n[state[nb].getEdId(state[i].p[2], state[i].p[1])] = tcInd;
            nb = state[i].n[2];
            if (nb != -1)
                state[nb].n[state[nb].getEdId(state[i].p[0], state[i].p[2])] = taInd;


            state[i].p[0] = -1;
        }
        else
        {
            uint32_t taInd = state.size();
            uint32_t tbInd = state.size() + 1;
            state.resize(state.size() + 2);

            int32_t bPoint = state[i].p[(cv + 2) % 3];

            state[taInd].p[0] = bPoint;
            state[taInd].p[1] = state[i].p[cv];
            state[taInd].p[2] = p;

            state[tbInd].p[0] = bPoint;
            state[tbInd].p[1] = p;
            state[tbInd].p[2] = state[i].p[(cv + 1) % 3];

            state[taInd].n[0] = state[i].n[(cv + 2) % 3];
            state[taInd].n[1] = -1;
            state[taInd].n[2] = tbInd;

            state[tbInd].n[0] = taInd;
            state[tbInd].n[1] = -1;
            state[tbInd].n[2] = state[i].n[(cv + 1) % 3];

            if (state[i].n[(cv + 1) % 3] != -1)
                for (int32_t k = 0; k < 3; ++k)
                    if (state[state[i].n[(cv + 1) % 3]].n[k] == (int32_t)i)
                    {
                        state[state[i].n[(cv + 1) % 3]].n[k] = tbInd;
                        break;
                    }
            if (state[i].n[(cv + 2) % 3] != -1)
                for (int32_t k = 0; k < 3; ++k)
                    if (state[state[i].n[(cv + 2) % 3]].n[k] == (int32_t)i)
                    {
                        state[state[i].n[(cv + 2) % 3]].n[k] = taInd;
                        break;
                    }

            triangleToCheck.push(taInd);
            triangleToCheck.push(tbInd);

            int32_t total      = 2;
            int32_t oppositeTr = 0;
            if (state[i].n[cv] != -1)
            {
                oppositeTr = state[i].n[cv];
                total += 2;
                uint32_t tcInd = state.size();
                uint32_t tdInd = state.size() + 1;
                state.resize(state.size() + 2);

                int32_t oped = state[oppositeTr].getEdId(state[i].p[(cv + 1) % 3], state[i].p[cv]);


                state[tcInd].n[0] = state[oppositeTr].n[(oped + 2) % 3];
                state[tcInd].n[1] = tbInd;
                state[tbInd].n[1] = tcInd;
                state[tcInd].n[2] = tdInd;

                state[tdInd].n[0] = tcInd;
                state[tdInd].n[1] = taInd;
                state[taInd].n[1] = tdInd;
                state[tdInd].n[2] = state[oppositeTr].n[(oped + 1) % 3];
                if (state[oppositeTr].n[(oped + 2) % 3] != -1)
                    for (int32_t k = 0; k < 3; ++k)
                        if (state[state[oppositeTr].n[(oped + 2) % 3]].n[k] == oppositeTr)
                        {
                            state[state[oppositeTr].n[(oped + 2) % 3]].n[k] = tcInd;
                            break;
                        }
                if (state[oppositeTr].n[(oped + 1) % 3] != -1)
                    for (int32_t k = 0; k < 3; ++k)
                        if (state[state[oppositeTr].n[(oped + 1) % 3]].n[k] == oppositeTr)
                        {
                            state[state[oppositeTr].n[(oped + 1) % 3]].n[k] = tdInd;
                            break;
                        }

                int32_t pop       = state[oppositeTr].p[(oped + 2) % 3];
                state[tcInd].p[0] = pop;
                state[tcInd].p[1] = state[i].p[(cv + 1) % 3];
                state[tcInd].p[2] = p;

                state[tdInd].p[0] = pop;
                state[tdInd].p[1] = p;
                state[tdInd].p[2] = state[i].p[cv];


                state[oppositeTr].p[0] = -1;
                triangleToCheck.push(tcInd);
                triangleToCheck.push(tdInd);
            }
            state[i].p[0] = -1;
        }
        break;
    }

    while (!triangleToCheck.empty())
    {
        int32_t ctrid = triangleToCheck.front();
        triangleToCheck.pop();
        DelTriangle& ctr = state[ctrid];
        int32_t oppTr    = -5;
        int32_t ced      = 0;
        for (uint32_t i = 0; i < 3; ++i)
        {
            if (ctr.p[i] != p && ctr.p[(i + 1) % 3] != p)
            {
                ced   = i;
                oppTr = ctr.n[i];
                break;
            }
        }
        if (oppTr == -1)
            continue;
        bool toCont = false;
        for (size_t i = 0; i < edges.size(); ++i)
        {
            if ((int32_t)edges[i].s == ctr.p[ced] && ctr.p[(ced + 1) % 3] == (int32_t)edges[i].e)
            {
                toCont = true;
                break;
            }
            if ((int32_t)edges[i].e == ctr.p[ced] && ctr.p[(ced + 1) % 3] == (int32_t)edges[i].s)
            {
                toCont = true;
                break;
            }
        }
        if (toCont)
            continue;


        DelTriangle& otr = state[oppTr];

        if (inCircumcircle(vertices[state[oppTr].p[0]], vertices[state[oppTr].p[1]], vertices[state[oppTr].p[2]],
                           vertices[p]) > 0)
        {
            int32_t notPIndx = 0;
            for (; notPIndx < 3; ++notPIndx)
            {
                if (otr.p[notPIndx] != ctr.p[0] && otr.p[notPIndx] != ctr.p[1] && otr.p[notPIndx] != ctr.p[2])
                    break;
            }

            int32_t oppCed = state[oppTr].getEdId(ctr.p[(ced + 1) % 3], ctr.p[ced]);

            int32_t ntr1 = ctrid, ntr2 = oppTr;

            DelTriangle nt1, nt2;
            nt1.p[0] = state[oppTr].p[notPIndx];
            nt1.p[1] = p;
            nt1.n[0] = ntr2;
            nt1.p[2] = ctr.p[ced];
            nt1.n[1] = ctr.n[(ced + 2) % 3];
            nt1.n[2] = otr.n[(oppCed + 1) % 3];

            if (nt1.n[2] != -1)
                for (uint32_t k = 0; k < 3; ++k)
                    if (state[nt1.n[2]].n[k] == oppTr)
                        state[nt1.n[2]].n[k] = ntr1;

            nt2.p[0] = p;
            nt2.p[1] = state[oppTr].p[notPIndx];
            nt2.n[0] = ntr1;
            nt2.p[2] = ctr.p[(ced + 1) % 3];
            nt2.n[1] = otr.n[(oppCed + 2) % 3];
            nt2.n[2] = ctr.n[(ced + 1) % 3];
            if (nt2.n[2] != -1)
                for (uint32_t k = 0; k < 3; ++k)
                    if (state[nt2.n[2]].n[k] == ctrid)
                        state[nt2.n[2]].n[k] = ntr2;
            state[ntr1] = nt1;
            state[ntr2] = nt2;
            triangleToCheck.push(ntr1);
            triangleToCheck.push(ntr2);
        }
    }
}

bool edgeIsIntersected(const RVec2& a, const RVec2& b, const RVec2& es, const RVec2& ee)
{
    RVec2 t           = b - a;
    cpp_rational temp = (es - a).cross(t) * (ee - a).cross(t);

    if (temp < 0)
    {
        t = es - ee;
        if ((a - ee).cross(t) * (b - ee).cross(t) <= 0)
            return true;
    }
    return false;
}

void triangulatePseudoPolygon(std::vector<RVec2>& vertices, int32_t ba, int32_t bb, std::vector<int32_t>& pseudo,
                              std::vector<DelTriangle>& output)
{
    if (pseudo.empty())
        return;

    int32_t c = 0;
    if (pseudo.size() > 1)
    {
        for (uint32_t i = 1; i < pseudo.size(); ++i)
        {
            if (inCircumcircle(vertices[ba], vertices[bb], vertices[pseudo[c]], vertices[pseudo[i]]) > 0)
            {
                c = i;
            }
        }
        std::vector<int32_t> toLeft;
        std::vector<int32_t> toRight;

        for (int32_t t = 0; t < c; ++t)
        {
            toLeft.push_back(pseudo[t]);
        }
        for (size_t t = c + 1; t < pseudo.size(); ++t)
        {
            toRight.push_back(pseudo[t]);
        }
        if (toLeft.size() > 0)
            triangulatePseudoPolygon(vertices, ba, pseudo[c], toLeft, output);
        if (toRight.size() > 0)
            triangulatePseudoPolygon(vertices, pseudo[c], bb, toRight, output);
    }
    output.push_back(DelTriangle());
    output.back().p[0] = ba;
    output.back().p[1] = bb;
    output.back().p[2] = pseudo[c];
}


void insertEdge(std::vector<RVec2>& vertices, std::vector<DelTriangle>& output, int32_t edBeg, int32_t edEnd)
{
    bool hasEdge = false;
    for (auto& it : output)
    {
        for (uint32_t i = 0; i < 3; ++i)
            if ((it.p[i] == edBeg || it.p[i] == edEnd) && (it.p[(i + 1) % 3] == edBeg || it.p[(i + 1) % 3] == edEnd))
            {
                hasEdge = true;
            }
    }
    if (hasEdge)
        return;

    int32_t startTriangle = -1;
    int32_t edg           = -1;
    for (uint32_t i = 0; i < output.size(); ++i)
    {
        if (output[i].p[0] == -1)
            continue;

        if (output[i].p[0] == edBeg || output[i].p[1] == edBeg || output[i].p[2] == edBeg)
        {
            edg = output[i].getEdWP(edBeg);
            if (edgeIsIntersected(vertices[edBeg], vertices[edEnd], vertices[output[i].p[edg]],
                                  vertices[output[i].p[(edg + 1) % 3]]))
            {
                startTriangle = i;
                break;
            }
        }
    }
    if (startTriangle == -1)
    {
        return;
    }
    int32_t cvertex = edBeg;

    std::vector<int32_t> pointsAboveEdge;
    std::vector<int32_t> pointsBelowEdge;

    RVec2 vec = vertices[edEnd] - vertices[edBeg];

    if (vec.cross(vertices[output[startTriangle].p[edg]] - vertices[edBeg]) > 0)
    {
        pointsAboveEdge.push_back(output[startTriangle].p[edg]);
        pointsBelowEdge.push_back(output[startTriangle].p[(edg + 1) % 3]);
    }
    else
    {
        pointsBelowEdge.push_back(output[startTriangle].p[edg]);
        pointsAboveEdge.push_back(output[startTriangle].p[(edg + 1) % 3]);
    }

    while (1)
    {
        DelTriangle& ctr     = output[startTriangle];
        int32_t oed          = ctr.getEdWP(cvertex);
        int32_t nextTriangle = ctr.n[oed];

        if (output[nextTriangle].p[0] == edEnd || output[nextTriangle].p[1] == edEnd || output[nextTriangle].p[2] == edEnd)
        {
            ctr.p[0]                  = -1;
            output[nextTriangle].p[0] = -1;
            break;
        }

        DelTriangle& otr = output[nextTriangle];
        int32_t opp      = otr.p[otr.getOppP(ctr.p[(oed + 1) % 3], ctr.p[oed % 3])];

        int32_t nextPoint = 0;
        if (vec.cross((vertices[opp] - vertices[edBeg])) > 0)
        {
            pointsAboveEdge.push_back(opp);
            if (vec.cross(vertices[ctr.p[(oed + 1) % 3]] - vertices[edBeg]) > 0)
            {
                nextPoint = ctr.p[(oed + 1) % 3];
            }
            else
            {
                nextPoint = ctr.p[oed];
            }
        }
        else
        {
            pointsBelowEdge.push_back(opp);

            if (vec.cross(vertices[ctr.p[(oed + 1) % 3]] - vertices[edBeg]) < 0)
            {
                nextPoint = ctr.p[(oed + 1) % 3];
            }
            else
            {
                nextPoint = ctr.p[oed];
            }
        }
        startTriangle = nextTriangle;
        cvertex       = nextPoint;
        ctr.p[0]      = -1;
    }
    triangulatePseudoPolygon(vertices, edBeg, edEnd, pointsAboveEdge, output);
    std::reverse(pointsBelowEdge.begin(), pointsBelowEdge.end());
    triangulatePseudoPolygon(vertices, edEnd, edBeg, pointsBelowEdge, output);
    reubildAdjacency(output);
}


void buildCDT(std::vector<RVec3>& vertices, std::vector<Edge>& edges, std::vector<DelTriangle>& output,
              ProjectionDirections dr)
{
    std::vector<DelTriangle> state;

    DelTriangle crt;
    std::vector<bool> added(vertices.size(), false);

    for (uint32_t i = 0; i < 3; ++i)
    {
        crt.p[i]          = edges[i].s;
        added[edges[i].s] = true;
        crt.n[i]          = -1;  // dont have neighbors;
    }
    state.push_back(crt);


    std::vector<RVec2> p2d(vertices.size());
    for (uint32_t i = 0; i < vertices.size(); ++i)
    {
        p2d[i] = getProjectedPointWithWinding(vertices[i], dr);
    }

    for (size_t i = 0; i < edges.size(); ++i)
    {
        if (!added[edges[i].s])
        {
            insertPoint(p2d, state, edges[i].s, edges);
            added[edges[i].s] = true;
        }
        if (!added[edges[i].e])
        {
            insertPoint(p2d, state, edges[i].e, edges);
            added[edges[i].e] = true;
        }
        if (edges[i].s != edges[i].e)
        {
            insertEdge(p2d, state, edges[i].s, edges[i].e);
        }
    }

    for (uint32_t t = 0; t < state.size(); ++t)
    {
        if (state[t].p[0] != -1)
        {
            output.push_back(state[t]);
        }
    }
}

int32_t intersectSegments(RVec3& s1, RVec3& e1, RVec3& s2, RVec3& e2, ProjectionDirections dir,
                          std::vector<cpp_rational>& t1v, std::vector<cpp_rational>& t2v);

void getTriangleIntersectionCoplanar(uint32_t tr1, uint32_t tr2, std::vector<std::vector<RVec3> >& stencil,
                                     ProjectionDirections dr)
{
    std::vector<cpp_rational> intr1[3];
    std::vector<cpp_rational> intr2[3];

    RVec3 p1[3];
    p1[0] = stencil[tr1][0];
    p1[1] = stencil[tr1][1];
    p1[2] = stencil[tr1][3];

    RVec3 p2[3];
    p2[0] = stencil[tr2][0];
    p2[1] = stencil[tr2][1];
    p2[2] = stencil[tr2][3];

    for (uint32_t i = 0; i < 3; ++i)
    {
        for (uint32_t j = 0; j < 3; ++j)
        {
            intersectSegments(p1[i], p1[(i + 1) % 3], p2[j], p2[(j + 1) % 3], dr, intr1[i], intr2[j]);
        }
    }

    int32_t inRel1[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        inRel1[i] = isPointInside(getProjectedPointWithWinding(p2[0], dr), getProjectedPointWithWinding(p2[1], dr),
                                  getProjectedPointWithWinding(p2[2], dr), getProjectedPointWithWinding(p1[i], dr));
    }

    int32_t inRel2[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        inRel2[i] = isPointInside(getProjectedPointWithWinding(p1[0], dr), getProjectedPointWithWinding(p1[1], dr),
                                  getProjectedPointWithWinding(p1[2], dr), getProjectedPointWithWinding(p2[i], dr));
    }

    for (uint32_t i = 0; i < 3; ++i)
    {
        if (inRel1[i] == INSIDE_TR && inRel1[(i + 1) % 3] == INSIDE_TR)
        {
            stencil[tr2].push_back(p1[i]);
            stencil[tr2].push_back(p1[(i + 1) % 3]);
        }
        else
        {
            if (inRel1[i] == INSIDE_TR && intr1[i].size() == 1)
            {
                stencil[tr2].push_back(p1[i]);
                stencil[tr2].push_back((p1[(i + 1) % 3] - p1[i]) * intr1[i][0] + p1[i]);
            }
            if (inRel1[(i + 1) % 3] == INSIDE_TR && intr1[i].size() == 1)
            {
                stencil[tr2].push_back(p1[(i + 1) % 3]);
                stencil[tr2].push_back((p1[(i + 1) % 3] - p1[i]) * intr1[i][0] + p1[i]);
            }
            if (intr1[i].size() == 2)
            {
                stencil[tr2].push_back((p1[(i + 1) % 3] - p1[i]) * intr1[i][0] + p1[i]);
                stencil[tr2].push_back((p1[(i + 1) % 3] - p1[i]) * intr1[i][1] + p1[i]);
            }
        }
    }

    for (uint32_t i = 0; i < 3; ++i)
    {
        if (inRel2[i] == INSIDE_TR && inRel2[(i + 1) % 3] == INSIDE_TR)
        {
            stencil[tr1].push_back(p2[i]);
            stencil[tr1].push_back(p2[(i + 1) % 3]);
        }
        else
        {
            if (inRel2[i] == INSIDE_TR && intr2[i].size() == 1)
            {
                stencil[tr1].push_back(p2[i]);
                stencil[tr1].push_back((p2[(i + 1) % 3] - p2[i]) * intr2[i][0] + p2[i]);
            }
            if (inRel2[(i + 1) % 3] == INSIDE_TR && intr2[i].size() == 1)
            {
                stencil[tr1].push_back(p2[(i + 1) % 3]);
                stencil[tr1].push_back((p2[(i + 1) % 3] - p2[i]) * intr2[i][0] + p2[i]);
            }
            if (intr2[i].size() == 2)
            {
                stencil[tr1].push_back((p2[(i + 1) % 3] - p2[i]) * intr2[i][0] + p2[i]);
                stencil[tr1].push_back((p2[(i + 1) % 3] - p2[i]) * intr2[i][1] + p2[i]);
            }
        }
    }
}


int32_t
getTriangleIntersection3d(uint32_t tr1, uint32_t tr2, std::vector<std::vector<RVec3> >& stencil, ProjectionDirections dr)
{
    RatPlane pl1(stencil[tr1][0], stencil[tr1][1], stencil[tr1][3]);
    if (pl1.n.isZero())
    {
        std::swap(tr1, tr2);
        pl1 = RatPlane(stencil[tr1][0], stencil[tr1][1], stencil[tr1][3]);
        if (pl1.n.isZero())
            return 0;
    }


    cpp_rational d1 = pl1.distance(stencil[tr2][0]);
    cpp_rational d2 = pl1.distance(stencil[tr2][1]);
    cpp_rational d3 = pl1.distance(stencil[tr2][3]);

    int32_t sd1 = d1.sign();
    int32_t sd2 = d2.sign();
    int32_t sd3 = d3.sign();


    if (sd1 == 0 && sd2 == 0 && sd3 == 0)
    {
        getTriangleIntersectionCoplanar(tr1, tr2, stencil, dr);
        return 0;
    }
    /**
    Never intersected
    */
    if (sd1 < 0 && sd2 < 0 && sd3 < 0)
        return 0;
    if (sd1 > 0 && sd2 > 0 && sd3 > 0)
        return 0;

    RVec3 tb0 = stencil[tr2][0];
    RVec3 tb1 = stencil[tr2][1];
    RVec3 tb2 = stencil[tr2][3];

    if (sd1 * sd3 > 0)
    {
        std::swap(tb1, tb2);
        std::swap(d2, d3);
    }
    else
    {
        if (sd2 * sd3 > 0)
        {
            std::swap(tb0, tb2);
            std::swap(d1, d3);
        }
        else
        {
            if (sd3 == 0 && sd1 * sd2 < 0)
            {
                std::swap(tb0, tb2);
                std::swap(d1, d3);
            }
        }
    }

    RatPlane pl2(stencil[tr2][0], stencil[tr2][1], stencil[tr2][3]);

    cpp_rational d21 = pl2.distance(stencil[tr1][0]);
    cpp_rational d22 = pl2.distance(stencil[tr1][1]);
    cpp_rational d23 = pl2.distance(stencil[tr1][3]);

    int32_t sd21 = d21.sign();
    int32_t sd22 = d22.sign();
    int32_t sd23 = d23.sign();

    if (sd21 < 0 && sd22 < 0 && sd23 < 0)
        return 0;
    if (sd21 > 0 && sd22 > 0 && sd23 > 0)
        return 0;


    RVec3 ta0 = stencil[tr1][0];
    RVec3 ta1 = stencil[tr1][1];
    RVec3 ta2 = stencil[tr1][3];


    if (sd21 * sd23 > 0)
    {
        std::swap(ta1, ta2);
        std::swap(d22, d23);
    }
    else
    {
        if (sd22 * sd23 > 0)
        {
            std::swap(ta0, ta2);
            std::swap(d21, d23);
        }
        else
        {
            if (sd23 == 0 && sd21 * sd22 < 0)
            {
                std::swap(ta0, ta2);
                std::swap(d21, d23);
            }
        }
    }
    //////////////////////////////////////////////////
    RVec3 dir = ta2 - ta0;

    cpp_rational dirPlaneDot = dir.dot(pl2.n);


    RVec3 pointOnIntersectionLine;
    if (dirPlaneDot != 0)
    {
        pointOnIntersectionLine = ta0 - dir * (d21 / dirPlaneDot);
    }
    else
    {
        pointOnIntersectionLine = ta0;
    }
    RVec3 interLineDir = pl1.n.cross(pl2.n);
    cpp_rational sqd   = interLineDir.dot(interLineDir);
    if (sqd.is_zero())
        return 0;

    cpp_rational t1p2      = (ta1 - pointOnIntersectionLine).dot(interLineDir) / sqd;
    cpp_rational t1p3      = (ta2 - pointOnIntersectionLine).dot(interLineDir) / sqd;
    cpp_rational t1p2param = t1p2;
    if (d22 != d23)
    {
        t1p2param = t1p2 + (t1p3 - t1p2) * (d22 / (d22 - d23));
    }

    t1p2                   = (tb0 - pointOnIntersectionLine).dot(interLineDir) / sqd;
    t1p3                   = (tb2 - pointOnIntersectionLine).dot(interLineDir) / sqd;
    cpp_rational t2p1param = t1p2;
    if (d1 != d3)
    {
        t2p1param = t1p2 + (t1p3 - t1p2) * d1 / (d1 - d3);
    }

    t1p2                   = (tb1 - pointOnIntersectionLine).dot(interLineDir) / sqd;
    cpp_rational t2p2param = t1p2;
    if (d2 != d3)
    {
        t2p2param = t1p2 + (t1p3 - t1p2) * d2 / (d2 - d3);
    }
    cpp_rational beg1 = 0;

    if (t1p2param < 0)
    {
        std::swap(beg1, t1p2param);
    }
    if (t2p2param < t2p1param)
    {
        std::swap(t2p2param, t2p1param);
    }
    cpp_rational minEnd = std::min(t1p2param, t2p2param);
    cpp_rational maxBeg = std::max(beg1, t2p1param);

    if (minEnd > maxBeg)
    {
        RVec3 p1 = pointOnIntersectionLine + interLineDir * maxBeg;
        RVec3 p2 = pointOnIntersectionLine + interLineDir * minEnd;

        stencil[tr1].push_back(p1);
        stencil[tr1].push_back(p2);

        stencil[tr2].push_back(p1);
        stencil[tr2].push_back(p2);
        return 1;
    }
    return 0;
}

int32_t intersectSegments(RVec3& s1, RVec3& e1, RVec3& s2, RVec3& e2, ProjectionDirections dir,
                          std::vector<cpp_rational>& t1v, std::vector<cpp_rational>& t2v)
{
    RVec2 s1p = getProjectedPointWithWinding(s1, dir);
    RVec2 e1p = getProjectedPointWithWinding(e1, dir);

    RVec2 s2p = getProjectedPointWithWinding(s2, dir);
    RVec2 e2p = getProjectedPointWithWinding(e2, dir);

    RVec2 dir1 = e1p - s1p;
    RVec2 dir2 = s2p - e2p;

    cpp_rational crs = dir1.cross(dir2);
    if (crs != 0)
    {
        cpp_rational c1 = s2p.x - s1p.x;
        cpp_rational c2 = s2p.y - s1p.y;

        cpp_rational det1 = c1 * dir2.y - c2 * dir2.x;
        cpp_rational det2 = dir1.x * c2 - dir1.y * c1;

        cpp_rational t1 = det1 / crs;
        cpp_rational t2 = det2 / crs;

        if (t1 > 0 && t1 < 1 && (t2 >= 0 && t2 <= 1))
        {
            t1v.push_back(t1);
        }
        if (t2 > 0 && t2 < 1 && (t1 >= 0 && t1 <= 1))
        {
            t2v.push_back(t2);
        }
    }
    else
    {
        if (dir1.cross(s2p - s1p) == 0)
        {
            if (dir1.x != 0)
            {
                cpp_rational t1 = (s2p.x - s1p.x) / dir1.x;
                cpp_rational t2 = (e2p.x - s1p.x) / dir1.x;
                if (t1 > 0 && t1 < 1)
                    t1v.push_back(t1);
                if (t2 > 0 && t2 < 1)
                    t1v.push_back(t2);
            }
            else
            {
                if (dir1.y != 0)
                {
                    cpp_rational t1 = (s2p.y - s1p.y) / dir1.y;
                    cpp_rational t2 = (e2p.y - s1p.y) / dir1.y;
                    if (t1 > 0 && t1 < 1)
                        t1v.push_back(t1);
                    if (t2 > 0 && t2 < 1)
                        t1v.push_back(t2);
                }
            }
        }
        if (dir2.cross(s1p - s2p) == 0)
        {
            dir2 = e2p - s2p;
            if (dir2.x != 0)
            {
                cpp_rational t1 = (s1p.x - s2p.x) / dir2.x;
                cpp_rational t2 = (e1p.x - s2p.x) / dir2.x;
                if (t1 > 0 && t1 < 1)
                    t2v.push_back(t1);
                if (t2 > 0 && t2 < 1)
                    t2v.push_back(t2);
            }
            else
            {
                if (dir2.y != 0)
                {
                    cpp_rational t1 = (s1p.y - s2p.y) / dir2.y;
                    cpp_rational t2 = (e1p.y - s2p.y) / dir2.y;
                    if (t1 > 0 && t1 < 1)
                        t2v.push_back(t1);
                    if (t2 > 0 && t2 < 1)
                        t2v.push_back(t2);
                }
            }
        }
    }
    return 1;
}

struct RVec3Comparer
{
    bool operator()(const RVec3& a, const RVec3& b) const
    {
        if (a.x < b.x)
            return true;
        if (a.x > b.x)
            return false;
        if (a.y < b.y)
            return true;
        if (a.y > b.y)
            return false;
        if (a.z < b.z)
            return true;
        return false;
    }
};

void getBarycentricCoords(NvVec2& a, NvVec2& b, NvVec2& c, NvVec2& p, float& u, float& v)
{
    NvVec3 v1(b.x - a.x, c.x - a.x, a.x - p.x);
    NvVec3 v2(b.y - a.y, c.y - a.y, a.y - p.y);

    NvVec3 resl = v1.cross(v2);
    u           = resl.x / resl.z;
    v           = resl.y / resl.z;
}


Mesh* MeshCleanerImpl::cleanMesh(const Mesh* mesh)
{
    /**
    ======= Get mesh data ===========
    */
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<Facet> facets;

    vertices.resize(mesh->getVerticesCount());
    edges.resize(mesh->getEdgesCount());
    facets.resize(mesh->getFacetCount());

    nvidia::NvBounds3 bnd;
    bnd.setEmpty();

    for (uint32_t i = 0; i < mesh->getVerticesCount(); ++i)
    {
        vertices[i] = mesh->getVertices()[i];
        bnd.include(toNvShared(vertices[i].p));
    }
    for (uint32_t i = 0; i < mesh->getEdgesCount(); ++i)
    {
        edges[i] = mesh->getEdges()[i];
    }
    for (uint32_t i = 0; i < mesh->getFacetCount(); ++i)
    {
        facets[i] = mesh->getFacetsBuffer()[i];
    }
    //======================================

    /**
        Transform vertices to fit unit cube and snap them to grid.
    **/
    float scale = 1.0f / bnd.getExtents().abs().maxElement();

    int32_t gridSize = 10000;  // Grid resolution to which vertices position will be snapped.

    for (uint32_t i = 0; i < mesh->getVerticesCount(); ++i)
    {
        vertices[i].p   = (vertices[i].p - fromNvShared(bnd.minimum)) * scale;
        vertices[i].p.x = std::floor(vertices[i].p.x * gridSize) / gridSize;
        vertices[i].p.y = std::floor(vertices[i].p.y * gridSize) / gridSize;
        vertices[i].p.z = std::floor(vertices[i].p.z * gridSize) / gridSize;
    }

    std::vector<std::vector<RVec3> > triangleStencil(facets.size());

    std::vector<NvVec3> facetsNormals(facets.size());
    std::vector<NvBounds3> facetBound(facets.size());


    for (uint32_t tr1 = 0; tr1 < facets.size(); ++tr1)
    {
        if (facets[tr1].edgesCount != 3)
        {
            return nullptr;
        }
        int32_t fed = facets[tr1].firstEdgeNumber;
        triangleStencil[tr1].push_back(vertices[edges[fed].s].p);
        triangleStencil[tr1].push_back(vertices[edges[fed].e].p);
        triangleStencil[tr1].push_back(vertices[edges[fed + 1].s].p);
        triangleStencil[tr1].push_back(vertices[edges[fed + 1].e].p);
        triangleStencil[tr1].push_back(vertices[edges[fed + 2].s].p);
        triangleStencil[tr1].push_back(vertices[edges[fed + 2].e].p);

        facetBound[tr1].setEmpty();
        facetBound[tr1].include(toNvShared(vertices[edges[fed].s].p));
        facetBound[tr1].include(toNvShared(vertices[edges[fed].e].p));
        facetBound[tr1].include(toNvShared(vertices[edges[fed + 2].s].p));
        facetBound[tr1].fattenFast(0.001f);

        facetsNormals[tr1] = toNvShared(vertices[edges[fed + 1].s].p - vertices[edges[fed].s].p)
                                 .cross(toNvShared(vertices[edges[fed + 2].s].p - vertices[edges[fed].s].p));
    }

    /**
        Build intersections between all pairs of triangles.
    */
    for (uint32_t tr1 = 0; tr1 < facets.size(); ++tr1)
    {
        if (triangleStencil[tr1].empty())
            continue;
        for (uint32_t tr2 = tr1 + 1; tr2 < facets.size(); ++tr2)
        {
            if (triangleStencil[tr2].empty())
                continue;
            if (facetBound[tr1].intersects(facetBound[tr2]) == false)
                continue;

            getTriangleIntersection3d(tr1, tr2, triangleStencil, getProjectionDirection(facetsNormals[tr1]));
        }
    }

    /**
    Reintersect all segments
    */
    for (uint32_t tr = 0; tr < triangleStencil.size(); ++tr)
    {
        std::vector<RVec3>& ctr = triangleStencil[tr];
        std::vector<std::vector<cpp_rational> > perSegmentInters(ctr.size() / 2);
        for (uint32_t sg1 = 6; sg1 < ctr.size(); sg1 += 2)
        {
            for (uint32_t sg2 = sg1 + 2; sg2 < ctr.size(); sg2 += 2)
            {
                intersectSegments(ctr[sg1], ctr[sg1 + 1], ctr[sg2], ctr[sg2 + 1],
                                  getProjectionDirection(facetsNormals[tr]), perSegmentInters[sg1 / 2],
                                  perSegmentInters[sg2 / 2]);
            }
        }

        std::vector<RVec3> newStencil;
        newStencil.reserve(ctr.size());

        for (uint32_t i = 0; i < ctr.size(); i += 2)
        {
            int32_t csm = i / 2;
            if (perSegmentInters[csm].size() == 0)
            {
                newStencil.push_back(ctr[i]);
                newStencil.push_back(ctr[i + 1]);
            }
            else
            {
                cpp_rational current = 0;
                newStencil.push_back(ctr[i]);
                std::sort(perSegmentInters[csm].begin(), perSegmentInters[csm].end());
                for (size_t j = 0; j < perSegmentInters[csm].size(); ++j)
                {
                    if (perSegmentInters[csm][j] > current)
                    {
                        current   = perSegmentInters[csm][j];
                        RVec3 pnt = (ctr[i + 1] - ctr[i]) * current + ctr[i];
                        newStencil.push_back(pnt);
                        newStencil.push_back(pnt);
                    }
                }
                newStencil.push_back(ctr[i + 1]);
            }
        }
        ctr = newStencil;
    }

    std::vector<RVec3> finalPoints;

    std::vector<std::vector<Edge> > tsten(facets.size());

    {
        std::map<RVec3, uint32_t, RVec3Comparer> mapping;
        for (uint32_t tr1 = 0; tr1 < triangleStencil.size(); ++tr1)
        {
            for (uint32_t j = 0; j < triangleStencil[tr1].size(); j += 2)
            {

                auto it    = mapping.find(triangleStencil[tr1][j]);
                int32_t pt = 0;
                if (it == mapping.end())
                {
                    mapping[triangleStencil[tr1][j]] = finalPoints.size();
                    pt                               = finalPoints.size();
                    finalPoints.push_back(triangleStencil[tr1][j]);
                }
                else
                {
                    pt = it->second;
                }

                Edge newed;

                newed.s = pt;

                it = mapping.find(triangleStencil[tr1][j + 1]);
                if (it == mapping.end())
                {
                    mapping[triangleStencil[tr1][j + 1]] = finalPoints.size();
                    pt                                   = finalPoints.size();
                    finalPoints.push_back(triangleStencil[tr1][j + 1]);
                }
                else
                {
                    pt = it->second;
                }
                newed.e         = pt;
                bool hasNewEdge = false;
                for (uint32_t e = 0; e < tsten[tr1].size(); ++e)
                {
                    if (tsten[tr1][e].s == newed.s && tsten[tr1][e].e == newed.e)
                    {
                        hasNewEdge = true;
                        break;
                    }
                    if (tsten[tr1][e].e == newed.s && tsten[tr1][e].s == newed.e)
                    {
                        hasNewEdge = true;
                        break;
                    }
                }
                if (!hasNewEdge)
                    tsten[tr1].push_back(newed);
            }
        }
    }

    /**
        Build constrained DT
    */
    std::vector<DelTriangle> trs;
    for (uint32_t i = 0; i < tsten.size(); ++i)
    {

        if (tsten[i].size() < 3)
            continue;
        if (tsten[i].size() > 3)
        {
            int32_t oldSize = trs.size();
            buildCDT(finalPoints, tsten[i], trs, getProjectionDirection(facetsNormals[i]));
            for (uint32_t k = oldSize; k < trs.size(); ++k)
                trs[k].parentTriangle = i;
        }
        else
        {
            trs.push_back(DelTriangle());
            trs.back().parentTriangle = i;
            for (uint32_t v = 0; v < 3; ++v)
                trs.back().p[v] = tsten[i][v].s;
        }
    }

    /**
        Remove 'deleted' triangles from array.
    */
    {
        std::vector<DelTriangle> trstemp;
        trstemp.reserve(trs.size());
        for (uint32_t i = 0; i < trs.size(); ++i)
        {
            if (trs[i].p[0] != -1)
                trstemp.push_back(trs[i]);
        }
        trs = trstemp;
    }

    /**
        Filter exterior surface
    */
    std::vector<bool> fillingMask(trs.size(), false);

    std::map<std::pair<int32_t, int32_t>, int32_t> edgeMap;
    std::vector<std::vector<int32_t> > edgeToTriangleMapping;

    for (uint32_t i = 0; i < trs.size(); ++i)
    {
        if (trs[i].p[0] == -1)
            continue;
        if (trs[i].p[0] == trs[i].p[1] || trs[i].p[2] == trs[i].p[1] || trs[i].p[2] == trs[i].p[0])
        {
            trs[i].p[0] = -1;
            continue;
        }
#if 0  // Filter null-area triangles.
        if ((finalPoints[trs[i].p[1]] - finalPoints[trs[i].p[0]]).cross(finalPoints[trs[i].p[2]] - finalPoints[trs[i].p[0]]).isZero())
        {
        trs[i].p[0] = -1;
        continue;
        }
#endif
        for (uint32_t k = 0; k < 3; ++k)
        {
            int32_t es = trs[i].p[k];
            int32_t ee = trs[i].p[(k + 1) % 3];
            if (es > ee)
            {
                std::swap(es, ee);
            }
            auto pr   = std::make_pair(es, ee);
            auto iter = edgeMap.find(pr);
            if (iter == edgeMap.end())
            {
                edgeMap[pr] = edgeToTriangleMapping.size();
                trs[i].n[k] = edgeToTriangleMapping.size();
                edgeToTriangleMapping.resize(edgeToTriangleMapping.size() + 1);
                edgeToTriangleMapping.back().push_back(i);
            }
            else
            {
                for (uint32_t j = 0; j < edgeToTriangleMapping[iter->second].size(); ++j)
                {
                    if (trs[edgeToTriangleMapping[iter->second][j]].compare(trs[i]))
                    {
                        trs[i].p[0] = -1;
                        break;
                    }
                }
                if (trs[i].p[0] != -1)
                {
                    trs[i].n[k] = iter->second;
                    edgeToTriangleMapping[iter->second].push_back(i);
                }
            }
        }
    }

    std::queue<int32_t> trque;
    float maxx   = -1000;
    int32_t best = 0;
    for (uint32_t i = 0; i < trs.size(); ++i)
    {
        if (trs[i].p[0] == -1)
            continue;
        float m = std::max(
            finalPoints[trs[i].p[0]].x.convert_to<float>(),
            std::max(finalPoints[trs[i].p[1]].x.convert_to<float>(), finalPoints[trs[i].p[2]].x.convert_to<float>()));
        if (m > maxx && facetsNormals[trs[i].parentTriangle].x > 0)
        {
            maxx = m;
            best = i;
        }
    }

    if (!trs.empty())
    {
        trque.push(best);
    }

    while (!trque.empty())
    {
        int32_t trid      = trque.front();
        fillingMask[trid] = true;
        DelTriangle& tr   = trs[trque.front()];
        trque.pop();

        for (uint32_t ed = 0; ed < 3; ++ed)
        {
            auto& tlist = edgeToTriangleMapping[tr.n[ed]];
            if (tlist.size() == 2)
            {
                for (uint32_t k = 0; k < tlist.size(); ++k)
                {
                    int32_t to = tlist[k];
                    if (to != trid && !fillingMask[to] && edgeToTriangleMapping[trs[to].n[0]].size() > 0 &&
                        edgeToTriangleMapping[trs[to].n[1]].size() > 0 && edgeToTriangleMapping[trs[to].n[2]].size() > 0)
                    {
                        trque.push(tlist[k]);
                        fillingMask[tlist[k]] = true;
                    }
                }
            }
            if (tlist.size() > 2)
            {
                int32_t bestPath = (tlist[0] == trid) ? tlist[1] : tlist[0];
                RVec3 start      = finalPoints[trs[trid].p[ed]];
                RVec3 axis       = finalPoints[trs[trid].p[(ed + 1) % 3]] - start;
                RVec3 nAxis      = finalPoints[trs[trid].p[(ed + 2) % 3]] - start;
                RVec3 normal     = axis.cross(nAxis);


                uint32_t op = trs[bestPath].getOppPoint(trs[trid].p[ed], trs[trid].p[(ed + 1) % 3]);

                RVec3 dir2           = (finalPoints[op] - start);
                RVec3 normal2        = dir2.cross(axis);
                cpp_rational bestDir = normal.cross(normal2).dot(axis);
                cpp_rational oldDist = normal2.dot(normal2);
                for (uint32_t k = 0; k < tlist.size(); ++k)
                {
                    if (tlist[k] == trid)
                        continue;
                    op                  = trs[tlist[k]].getOppPoint(trs[trid].p[ed], trs[trid].p[(ed + 1) % 3]);
                    dir2                = (finalPoints[op] - start);
                    normal2             = dir2.cross(axis);
                    cpp_rational newOne = normal.cross(normal2).dot(axis);

                    if (newOne * oldDist < bestDir * normal2.dot(normal2))
                    {
                        oldDist  = normal2.dot(normal2);
                        bestPath = tlist[k];
                        bestDir  = newOne;
                    }
                }
                if (!fillingMask[bestPath] && edgeToTriangleMapping[trs[bestPath].n[0]].size() > 0 &&
                    edgeToTriangleMapping[trs[bestPath].n[1]].size() > 0 &&
                    edgeToTriangleMapping[trs[bestPath].n[2]].size() > 0)
                {
                    trque.push(bestPath);
                    fillingMask[bestPath] = true;
                }
            }
            edgeToTriangleMapping[tr.n[ed]].clear();
        }
    }
    for (uint32_t id = 0; id < trs.size(); ++id)
    {
        if (!fillingMask[id])
        {
            trs[id].p[0] = -1;  // Remove triangle
        }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<NvVec3> newVertices;
    newVertices.resize(finalPoints.size());
    for (uint32_t i = 0; i < finalPoints.size(); ++i)
    {
        newVertices[i].x = finalPoints[i].x.convert_to<float>();
        newVertices[i].y = finalPoints[i].y.convert_to<float>();
        newVertices[i].z = finalPoints[i].z.convert_to<float>();
    }
    /**
        Rescale mesh to initial coordinates.
    */
    for (uint32_t i = 0; i < finalPoints.size(); ++i)
    {
        newVertices[i] = newVertices[i] * (1.0f / scale) + bnd.minimum;
    }
    for (uint32_t i = 0; i < vertices.size(); ++i)
    {
        vertices[i].p = vertices[i].p * (1.0f / scale) + fromNvShared(bnd.minimum);
    }

    std::vector<Triangle> result;
    result.reserve(trs.size());
    {
        std::vector<NvVec2> projectedTriangles(facets.size() * 3);
        std::vector<Vertex> normalTriangles(facets.size() * 3);

        for (uint32_t i = 0; i < facets.size(); ++i)
        {
            for (uint32_t k = 0; k < 3; ++k)
            {
                normalTriangles[i * 3 + k]    = vertices[edges[facets[i].firstEdgeNumber + k].s];
                projectedTriangles[i * 3 + k] = getProjectedPointWithWinding(
                    vertices[edges[facets[i].firstEdgeNumber + k].s].p, getProjectionDirection(facetsNormals[i])).toVec2();
            }
        }

        for (uint32_t i = 0; i < trs.size(); ++i)
        {
            if (trs[i].p[0] == -1)
                continue;
            int32_t id             = 0;
            int32_t parentTriangle = trs[i].parentTriangle;
            float u = 0, v = 0;
            result.resize(result.size() + 1);
            result.back().materialId     = facets[parentTriangle].materialId;
            result.back().smoothingGroup = facets[parentTriangle].smoothingGroup;
            for (auto vert : { &result.back().a, &result.back().b, &result.back().c })
            {
                toNvShared(vert->p)  = newVertices[trs[i].p[id]];
                NvVec2 p = getProjectedPointWithWinding(vert->p, getProjectionDirection(facetsNormals[parentTriangle])).toVec2();
                getBarycentricCoords(projectedTriangles[parentTriangle * 3], projectedTriangles[parentTriangle * 3 + 1],
                                     projectedTriangles[parentTriangle * 3 + 2], p, u, v);
                vert->uv[0] = (1 - u - v) * normalTriangles[parentTriangle * 3].uv[0] +
                              u * normalTriangles[parentTriangle * 3 + 1].uv[0] +
                              v * normalTriangles[parentTriangle * 3 + 2].uv[0];
                vert->n = (1 - u - v) * normalTriangles[parentTriangle * 3].n +
                          u * normalTriangles[parentTriangle * 3 + 1].n + v * normalTriangles[parentTriangle * 3 + 2].n;
                ++id;
            }
        }
    }

    /**
        Reuse old buffers to create Mesh
    */
    std::vector<NvcVec3> newMeshVertices(result.size() * 3);
    std::vector<NvcVec3> newMeshNormals(result.size() * 3);
    std::vector<NvcVec2> newMeshUvs(result.size() * 3);

    std::vector<int32_t> newMaterialIds(result.size());
    std::vector<int32_t> newSmoothingGroups(result.size());


    for (uint32_t i = 0; i < result.size(); ++i)
    {
        Vertex* arr[3] = { &result[i].a, &result[i].b, &result[i].c };
        for (uint32_t k = 0; k < 3; ++k)
        {
            newMeshVertices[i * 3 + k] = arr[k]->p;
            newMeshNormals[i * 3 + k]  = arr[k]->n;
            newMeshUvs[i * 3 + k]      = arr[k]->uv[0];
        }
    }
    std::vector<uint32_t> serializedIndices;
    serializedIndices.reserve(result.size() * 3);
    int32_t cindex = 0;
    for (uint32_t i = 0; i < result.size(); ++i)
    {
        newMaterialIds[i]     = result[i].materialId;
        newSmoothingGroups[i] = result[i].smoothingGroup;

        for (uint32_t pi = 0; pi < 3; ++pi)
            serializedIndices.push_back(cindex++);
    }

    MeshImpl* rMesh = new MeshImpl(newMeshVertices.data(), newMeshNormals.data(), newMeshUvs.data(),
                                   static_cast<uint32_t>(newMeshVertices.size()), serializedIndices.data(),
                                   static_cast<uint32_t>(serializedIndices.size()));
    rMesh->setMaterialId(newMaterialIds.data());
    rMesh->setSmoothingGroup(newSmoothingGroups.data());
    return rMesh;
}

void MeshCleanerImpl::release()
{
    delete this;
}
