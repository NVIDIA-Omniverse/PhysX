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


#include "NvBlastExtApexSharedParts.h"

#include "NvBlastGlobals.h"
#include "NvBlastMemory.h"
#include "NvBlastAssert.h"

#include "NsVecMath.h"

#include "NvMat44.h"
#include "NvBounds3.h"
#include "NsVecMath.h"
#include <vector>

using namespace nvidia;
using namespace nvidia::shdfnd::aos;


namespace Nv
{
namespace Blast
{

NV_NOALIAS NV_FORCE_INLINE BoolV PointOutsideOfPlane4(const Vec3VArg _a, const Vec3VArg _b, const Vec3VArg _c, const Vec3VArg _d)
{
    // this is not 0 because of the following scenario:
    // All the points lie on the same plane and the plane goes through the origin (0,0,0).
    // On the Wii U, the math below has the problem that when point A gets projected on the
    // plane cumputed by A, B, C, the distance to the plane might not be 0 for the mentioned
    // scenario but a small positive or negative value. This can lead to the wrong boolean
    // results. Using a small negative value as threshold is more conservative but safer.
    const Vec4V zero = V4Load(-1e-6f);

    const Vec3V ab = V3Sub(_b, _a);
    const Vec3V ac = V3Sub(_c, _a);
    const Vec3V ad = V3Sub(_d, _a);
    const Vec3V bd = V3Sub(_d, _b);
    const Vec3V bc = V3Sub(_c, _b);

    const Vec3V v0 = V3Cross(ab, ac);
    const Vec3V v1 = V3Cross(ac, ad);
    const Vec3V v2 = V3Cross(ad, ab);
    const Vec3V v3 = V3Cross(bd, bc);

    const FloatV signa0 = V3Dot(v0, _a);
    const FloatV signa1 = V3Dot(v1, _a);
    const FloatV signa2 = V3Dot(v2, _a);
    const FloatV signd3 = V3Dot(v3, _a);

    const FloatV signd0 = V3Dot(v0, _d);
    const FloatV signd1 = V3Dot(v1, _b);
    const FloatV signd2 = V3Dot(v2, _c);
    const FloatV signa3 = V3Dot(v3, _b);

    const Vec4V signa = V4Merge(signa0, signa1, signa2, signa3);
    const Vec4V signd = V4Merge(signd0, signd1, signd2, signd3);
    return V4IsGrtrOrEq(V4Mul(signa, signd), zero);//same side, outside of the plane
}

NV_NOALIAS NV_FORCE_INLINE Vec3V closestPtPointSegment(const Vec3VArg a, const Vec3VArg b)
{
    const FloatV zero = FZero();
    const FloatV one = FOne();

    //Test degenerated case
    const Vec3V ab = V3Sub(b, a);
    const FloatV denom = V3Dot(ab, ab);
    const Vec3V ap = V3Neg(a);//V3Sub(origin, a);
    const FloatV nom = V3Dot(ap, ab);
    const BoolV con = FIsEq(denom, zero);
    const FloatV tValue = FClamp(FDiv(nom, denom), zero, one);
    const FloatV t = FSel(con, zero, tValue);

    return V3Sel(con, a, V3ScaleAdd(ab, t, a));
}

NV_NOALIAS NV_FORCE_INLINE Vec3V closestPtPointSegment(const Vec3VArg Q0, const Vec3VArg Q1, const Vec3VArg A0, const Vec3VArg A1,
    const Vec3VArg B0, const Vec3VArg B1, uint32_t& size, Vec3V& closestA, Vec3V& closestB)
{
    const Vec3V a = Q0;
    const Vec3V b = Q1;

    const BoolV bTrue = BTTTT();
    const FloatV zero = FZero();
    const FloatV one = FOne();

    //Test degenerated case
    const Vec3V ab = V3Sub(b, a);
    const FloatV denom = V3Dot(ab, ab);
    const Vec3V ap = V3Neg(a);//V3Sub(origin, a);
    const FloatV nom = V3Dot(ap, ab);
    const BoolV con = FIsEq(denom, zero);

    if (BAllEq(con, bTrue))
    {
        size = 1;
        closestA = A0;
        closestB = B0;
        return Q0;
    }

    const Vec3V v = V3Sub(A1, A0);
    const Vec3V w = V3Sub(B1, B0);
    const FloatV tValue = FClamp(FDiv(nom, denom), zero, one);
    const FloatV t = FSel(con, zero, tValue);

    const Vec3V tempClosestA = V3ScaleAdd(v, t, A0);
    const Vec3V tempClosestB = V3ScaleAdd(w, t, B0);
    closestA = tempClosestA;
    closestB = tempClosestB;
    return V3Sub(tempClosestA, tempClosestB);
}

NV_NOALIAS Vec3V closestPtPointSegmentTesselation(const Vec3VArg Q0, const Vec3VArg Q1, const Vec3VArg A0, const Vec3VArg A1,
    const Vec3VArg B0, const Vec3VArg B1, uint32_t& size, Vec3V& closestA, Vec3V& closestB)
{
    const FloatV half = FHalf();

    const FloatV targetSegmentLengthSq = FLoad(10000.f);//100 unit

    Vec3V q0 = Q0;
    Vec3V q1 = Q1;
    Vec3V a0 = A0;
    Vec3V a1 = A1;
    Vec3V b0 = B0;
    Vec3V b1 = B1;

    for (;;)
    {
        const Vec3V midPoint = V3Scale(V3Add(q0, q1), half);
        const Vec3V midA = V3Scale(V3Add(a0, a1), half);
        const Vec3V midB = V3Scale(V3Add(b0, b1), half);

        const Vec3V v = V3Sub(midPoint, q0);
        const FloatV sqV = V3Dot(v, v);
        if (FAllGrtr(targetSegmentLengthSq, sqV))
            break;
        //split the segment into half
        const Vec3V tClos0 = closestPtPointSegment(q0, midPoint);
        const FloatV sqDist0 = V3Dot(tClos0, tClos0);

        const Vec3V tClos1 = closestPtPointSegment(q1, midPoint);
        const FloatV sqDist1 = V3Dot(tClos1, tClos1);
        //const BoolV con = FIsGrtr(sqDist0, sqDist1);
        if (FAllGrtr(sqDist0, sqDist1))
        {
            //segment [m, q1]
            q0 = midPoint;
            a0 = midA;
            b0 = midB;
        }
        else
        {
            //segment [q0, m]
            q1 = midPoint;
            a1 = midA;
            b1 = midB;
        }

    }

    return closestPtPointSegment(q0, q1, a0, a1, b0, b1, size, closestA, closestB);
}

NV_NOALIAS Vec3V closestPtPointTriangleTesselation(const Vec3V* NV_RESTRICT Q, const Vec3V* NV_RESTRICT A, const Vec3V* NV_RESTRICT B, const uint32_t* NV_RESTRICT indices, uint32_t& size, Vec3V& closestA, Vec3V& closestB)
{
    size = 3;
    const FloatV zero = FZero();
    const FloatV eps = FEps();
    const FloatV half = FHalf();
    const BoolV bTrue = BTTTT();
    const FloatV four = FLoad(4.f);
    const FloatV sixty = FLoad(100.f);

    const uint32_t ind0 = indices[0];
    const uint32_t ind1 = indices[1];
    const uint32_t ind2 = indices[2];

    const Vec3V a = Q[ind0];
    const Vec3V b = Q[ind1];
    const Vec3V c = Q[ind2];

    Vec3V ab_ = V3Sub(b, a);
    Vec3V ac_ = V3Sub(c, a);
    Vec3V bc_ = V3Sub(b, c);

    const FloatV dac_ = V3Dot(ac_, ac_);
    const FloatV dbc_ = V3Dot(bc_, bc_);
    if (FAllGrtrOrEq(eps, FMin(dac_, dbc_)))
    {
        //degenerate
        size = 2;
        return closestPtPointSegment(Q[ind0], Q[ind1], A[ind0], A[ind1], B[ind0], B[ind1], size, closestA, closestB);
    }

    Vec3V ap = V3Neg(a);
    Vec3V bp = V3Neg(b);
    Vec3V cp = V3Neg(c);

    FloatV d1 = V3Dot(ab_, ap); //  snom
    FloatV d2 = V3Dot(ac_, ap); //  tnom
    FloatV d3 = V3Dot(ab_, bp); // -sdenom
    FloatV d4 = V3Dot(ac_, bp); //  unom = d4 - d3
    FloatV d5 = V3Dot(ab_, cp); //  udenom = d5 - d6
    FloatV d6 = V3Dot(ac_, cp); // -tdenom
    /*  FloatV unom = FSub(d4, d3);
    FloatV udenom = FSub(d5, d6);*/

    FloatV va = FNegScaleSub(d5, d4, FMul(d3, d6));//edge region of BC
    FloatV vb = FNegScaleSub(d1, d6, FMul(d5, d2));//edge region of AC
    FloatV vc = FNegScaleSub(d3, d2, FMul(d1, d4));//edge region of AB

    //check if p in vertex region outside a
    const BoolV con00 = FIsGrtrOrEq(zero, d1); // snom <= 0
    const BoolV con01 = FIsGrtrOrEq(zero, d2); // tnom <= 0
    const BoolV con0 = BAnd(con00, con01); // vertex region a
    if (BAllEq(con0, bTrue))
    {
        //size = 1;
        closestA = A[ind0];
        closestB = B[ind0];
        return Q[ind0];
    }

    //check if p in vertex region outside b
    const BoolV con10 = FIsGrtrOrEq(d3, zero);
    const BoolV con11 = FIsGrtrOrEq(d3, d4);
    const BoolV con1 = BAnd(con10, con11); // vertex region b
    if (BAllEq(con1, bTrue))
    {
        /*size = 1;
        indices[0] = ind1;*/
        closestA = A[ind1];
        closestB = B[ind1];
        return Q[ind1];
    }


    //check if p in vertex region outside of c
    const BoolV con20 = FIsGrtrOrEq(d6, zero);
    const BoolV con21 = FIsGrtrOrEq(d6, d5);
    const BoolV con2 = BAnd(con20, con21); // vertex region c
    if (BAllEq(con2, bTrue))
    {
        closestA = A[ind2];
        closestB = B[ind2];
        return Q[ind2];
    }

    //check if p in edge region of AB
    const BoolV con30 = FIsGrtrOrEq(zero, vc);
    const BoolV con31 = FIsGrtrOrEq(d1, zero);
    const BoolV con32 = FIsGrtrOrEq(zero, d3);
    const BoolV con3 = BAnd(con30, BAnd(con31, con32));

    if (BAllEq(con3, bTrue))
    {
        //size = 2;
        //p in edge region of AB, split AB
        return closestPtPointSegmentTesselation(Q[ind0], Q[ind1], A[ind0], A[ind1], B[ind0], B[ind1], size, closestA, closestB);
    }

    //check if p in edge region of BC
    const BoolV con40 = FIsGrtrOrEq(zero, va);
    const BoolV con41 = FIsGrtrOrEq(d4, d3);
    const BoolV con42 = FIsGrtrOrEq(d5, d6);
    const BoolV con4 = BAnd(con40, BAnd(con41, con42));

    if (BAllEq(con4, bTrue))
    {
        //p in edge region of BC, split BC
        return closestPtPointSegmentTesselation(Q[ind1], Q[ind2], A[ind1], A[ind2], B[ind1], B[ind2], size, closestA, closestB);
    }

    //check if p in edge region of AC
    const BoolV con50 = FIsGrtrOrEq(zero, vb);
    const BoolV con51 = FIsGrtrOrEq(d2, zero);
    const BoolV con52 = FIsGrtrOrEq(zero, d6);
    const BoolV con5 = BAnd(con50, BAnd(con51, con52));

    if (BAllEq(con5, bTrue))
    {
        //p in edge region of AC, split AC
        return closestPtPointSegmentTesselation(Q[ind0], Q[ind2], A[ind0], A[ind2], B[ind0], B[ind2], size, closestA, closestB);
    }

    size = 3;

    Vec3V q0 = Q[ind0];
    Vec3V q1 = Q[ind1];
    Vec3V q2 = Q[ind2];
    Vec3V a0 = A[ind0];
    Vec3V a1 = A[ind1];
    Vec3V a2 = A[ind2];
    Vec3V b0 = B[ind0];
    Vec3V b1 = B[ind1];
    Vec3V b2 = B[ind2];

    for (;;)
    {

        const Vec3V ab = V3Sub(q1, q0);
        const Vec3V ac = V3Sub(q2, q0);
        const Vec3V bc = V3Sub(q2, q1);

        const FloatV dab = V3Dot(ab, ab);
        const FloatV dac = V3Dot(ac, ac);
        const FloatV dbc = V3Dot(bc, bc);

        const FloatV fMax = FMax(dab, FMax(dac, dbc));
        const FloatV fMin = FMin(dab, FMin(dac, dbc));

        const Vec3V w = V3Cross(ab, ac);

        const FloatV area = V3Length(w);
        const FloatV ratio = FDiv(FSqrt(fMax), FSqrt(fMin));
        if (FAllGrtr(four, ratio) && FAllGrtr(sixty, area))
            break;

        //calculate the triangle normal
        const Vec3V triNormal = V3Normalize(w);

        NVBLAST_ASSERT(V3AllEq(triNormal, V3Zero()) == 0);


        //split the longest edge
        if (FAllGrtrOrEq(dab, dac) && FAllGrtrOrEq(dab, dbc))
        {
            //split edge q0q1
            const Vec3V midPoint = V3Scale(V3Add(q0, q1), half);
            const Vec3V midA = V3Scale(V3Add(a0, a1), half);
            const Vec3V midB = V3Scale(V3Add(b0, b1), half);

            const Vec3V v = V3Sub(midPoint, q2);
            const Vec3V n = V3Normalize(V3Cross(v, triNormal));

            const FloatV d = FNeg(V3Dot(n, midPoint));
            const FloatV dp = FAdd(V3Dot(n, q0), d);
            const FloatV sum = FMul(d, dp);

            if (FAllGrtr(sum, zero))
            {
                //q0 and origin at the same side, split triangle[q0, m, q2]
                q1 = midPoint;
                a1 = midA;
                b1 = midB;
            }
            else
            {
                //q1 and origin at the same side, split triangle[m, q1, q2]
                q0 = midPoint;
                a0 = midA;
                b0 = midB;
            }

        }
        else if (FAllGrtrOrEq(dac, dbc))
        {
            //split edge q0q2
            const Vec3V midPoint = V3Scale(V3Add(q0, q2), half);
            const Vec3V midA = V3Scale(V3Add(a0, a2), half);
            const Vec3V midB = V3Scale(V3Add(b0, b2), half);

            const Vec3V v = V3Sub(midPoint, q1);
            const Vec3V n = V3Normalize(V3Cross(v, triNormal));

            const FloatV d = FNeg(V3Dot(n, midPoint));
            const FloatV dp = FAdd(V3Dot(n, q0), d);
            const FloatV sum = FMul(d, dp);

            if (FAllGrtr(sum, zero))
            {
                //q0 and origin at the same side, split triangle[q0, q1, m]
                q2 = midPoint;
                a2 = midA;
                b2 = midB;
            }
            else
            {
                //q2 and origin at the same side, split triangle[m, q1, q2]
                q0 = midPoint;
                a0 = midA;
                b0 = midB;
            }
        }
        else
        {
            //split edge q1q2
            const Vec3V midPoint = V3Scale(V3Add(q1, q2), half);
            const Vec3V midA = V3Scale(V3Add(a1, a2), half);
            const Vec3V midB = V3Scale(V3Add(b1, b2), half);

            const Vec3V v = V3Sub(midPoint, q0);
            const Vec3V n = V3Normalize(V3Cross(v, triNormal));

            const FloatV d = FNeg(V3Dot(n, midPoint));
            const FloatV dp = FAdd(V3Dot(n, q1), d);
            const FloatV sum = FMul(d, dp);

            if (FAllGrtr(sum, zero))
            {
                //q1 and origin at the same side, split triangle[q0, q1, m]
                q2 = midPoint;
                a2 = midA;
                b2 = midB;
            }
            else
            {
                //q2 and origin at the same side, split triangle[q0, m, q2]
                q1 = midPoint;
                a1 = midA;
                b1 = midB;
            }


        }
    }

    //P must project inside face region. Compute Q using Barycentric coordinates
    ab_ = V3Sub(q1, q0);
    ac_ = V3Sub(q2, q0);
    ap = V3Neg(q0);
    bp = V3Neg(q1);
    cp = V3Neg(q2);

    d1 = V3Dot(ab_, ap); //  snom
    d2 = V3Dot(ac_, ap); //  tnom
    d3 = V3Dot(ab_, bp); // -sdenom
    d4 = V3Dot(ac_, bp); //  unom = d4 - d3
    d5 = V3Dot(ab_, cp); //  udenom = d5 - d6
    d6 = V3Dot(ac_, cp); // -tdenom

    va = FNegScaleSub(d5, d4, FMul(d3, d6));//edge region of BC
    vb = FNegScaleSub(d1, d6, FMul(d5, d2));//edge region of AC
    vc = FNegScaleSub(d3, d2, FMul(d1, d4));//edge region of AB

    const FloatV toRecipD = FAdd(va, FAdd(vb, vc));
    const FloatV denom = FRecip(toRecipD);//V4GetW(recipTmp);
    const Vec3V v0 = V3Sub(a1, a0);
    const Vec3V v1 = V3Sub(a2, a0);
    const Vec3V w0 = V3Sub(b1, b0);
    const Vec3V w1 = V3Sub(b2, b0);

    const FloatV t = FMul(vb, denom);
    const FloatV w = FMul(vc, denom);
    const Vec3V vA1 = V3Scale(v1, w);
    const Vec3V vB1 = V3Scale(w1, w);
    const Vec3V tempClosestA = V3Add(a0, V3ScaleAdd(v0, t, vA1));
    const Vec3V tempClosestB = V3Add(b0, V3ScaleAdd(w0, t, vB1));
    closestA = tempClosestA;
    closestB = tempClosestB;
    return V3Sub(tempClosestA, tempClosestB);
}

NV_NOALIAS Vec3V closestPtPointTetrahedronTesselation(Vec3V* NV_RESTRICT Q, Vec3V* NV_RESTRICT A, Vec3V* NV_RESTRICT B, uint32_t& size, Vec3V& closestA, Vec3V& closestB)
{
    const FloatV eps = FEps();
    const Vec3V zeroV = V3Zero();
    uint32_t tempSize = size;

    FloatV bestSqDist = FLoad(NV_MAX_F32);
    const Vec3V a = Q[0];
    const Vec3V b = Q[1];
    const Vec3V c = Q[2];
    const Vec3V d = Q[3];
    const BoolV bTrue = BTTTT();
    const BoolV bFalse = BFFFF();

    //degenerated
    const Vec3V ad = V3Sub(d, a);
    const Vec3V bd = V3Sub(d, b);
    const Vec3V cd = V3Sub(d, c);
    const FloatV dad = V3Dot(ad, ad);
    const FloatV dbd = V3Dot(bd, bd);
    const FloatV dcd = V3Dot(cd, cd);
    const FloatV fMin = FMin(dad, FMin(dbd, dcd));
    if (FAllGrtr(eps, fMin))
    {
        size = 3;
        uint32_t tempIndices[] = { 0, 1, 2 };
        return closestPtPointTriangleTesselation(Q, A, B, tempIndices, size, closestA, closestB);
    }

    Vec3V _Q[] = { Q[0], Q[1], Q[2], Q[3] };
    Vec3V _A[] = { A[0], A[1], A[2], A[3] };
    Vec3V _B[] = { B[0], B[1], B[2], B[3] };

    uint32_t indices[3] = { 0, 1, 2 };

    const BoolV bIsOutside4 = PointOutsideOfPlane4(a, b, c, d);

    if (BAllEq(bIsOutside4, bFalse))
    {
        //origin is inside the tetrahedron, we are done
        return zeroV;
    }

    Vec3V result = zeroV;
    Vec3V tempClosestA, tempClosestB;

    if (BAllEq(BGetX(bIsOutside4), bTrue))
    {

        uint32_t tempIndices[] = { 0, 1, 2 };
        uint32_t _size = 3;

        result = closestPtPointTriangleTesselation(_Q, _A, _B, tempIndices, _size, tempClosestA, tempClosestB);

        const FloatV sqDist = V3Dot(result, result);
        bestSqDist = sqDist;

        indices[0] = tempIndices[0];
        indices[1] = tempIndices[1];
        indices[2] = tempIndices[2];

        tempSize = _size;
        closestA = tempClosestA;
        closestB = tempClosestB;
    }

    if (BAllEq(BGetY(bIsOutside4), bTrue))
    {

        uint32_t tempIndices[] = { 0, 2, 3 };

        uint32_t _size = 3;

        const Vec3V q = closestPtPointTriangleTesselation(_Q, _A, _B, tempIndices, _size, tempClosestA, tempClosestB);

        const FloatV sqDist = V3Dot(q, q);
        const BoolV con = FIsGrtr(bestSqDist, sqDist);
        if (BAllEq(con, bTrue))
        {
            result = q;
            bestSqDist = sqDist;
            indices[0] = tempIndices[0];
            indices[1] = tempIndices[1];
            indices[2] = tempIndices[2];

            tempSize = _size;
            closestA = tempClosestA;
            closestB = tempClosestB;
        }
    }

    if (BAllEq(BGetZ(bIsOutside4), bTrue))
    {

        uint32_t tempIndices[] = { 0, 3, 1 };
        uint32_t _size = 3;

        const Vec3V q = closestPtPointTriangleTesselation(_Q, _A, _B, tempIndices, _size, tempClosestA, tempClosestB);

        const FloatV sqDist = V3Dot(q, q);
        const BoolV con = FIsGrtr(bestSqDist, sqDist);
        if (BAllEq(con, bTrue))
        {
            result = q;
            bestSqDist = sqDist;
            indices[0] = tempIndices[0];
            indices[1] = tempIndices[1];
            indices[2] = tempIndices[2];
            tempSize = _size;
            closestA = tempClosestA;
            closestB = tempClosestB;
        }

    }

    if (BAllEq(BGetW(bIsOutside4), bTrue))
    {

        uint32_t tempIndices[] = { 1, 3, 2 };
        uint32_t _size = 3;

        const Vec3V q = closestPtPointTriangleTesselation(_Q, _A, _B, tempIndices, _size, tempClosestA, tempClosestB);

        const FloatV sqDist = V3Dot(q, q);
        const BoolV con = FIsGrtr(bestSqDist, sqDist);

        if (BAllEq(con, bTrue))
        {
            result = q;
            bestSqDist = sqDist;

            indices[0] = tempIndices[0];
            indices[1] = tempIndices[1];
            indices[2] = tempIndices[2];

            tempSize = _size;
            closestA = tempClosestA;
            closestB = tempClosestB;
        }
    }

    A[0] = _A[indices[0]]; A[1] = _A[indices[1]]; A[2] = _A[indices[2]];
    B[0] = _B[indices[0]]; B[1] = _B[indices[1]]; B[2] = _B[indices[2]];
    Q[0] = _Q[indices[0]]; Q[1] = _Q[indices[1]]; Q[2] = _Q[indices[2]];


    size = tempSize;
    return result;
}

NV_NOALIAS NV_FORCE_INLINE Vec3V doTesselation(Vec3V* NV_RESTRICT Q, Vec3V* NV_RESTRICT A, Vec3V* NV_RESTRICT B,
    const Vec3VArg support, const Vec3VArg supportA, const Vec3VArg supportB, uint32_t& size, Vec3V& closestA, Vec3V& closestB)
{
    switch (size)
    {
    case 1:
    {
        closestA = supportA;
        closestB = supportB;
        return support;
    }
    case 2:
    {
        return closestPtPointSegmentTesselation(Q[0], support, A[0], supportA, B[0], supportB, size, closestA, closestB);
    }
    case 3:
    {

        uint32_t tempIndices[3] = { 0, 1, 2 };
        return closestPtPointTriangleTesselation(Q, A, B, tempIndices, size, closestA, closestB);
    }
    case 4:
    {
        return closestPtPointTetrahedronTesselation(Q, A, B, size, closestA, closestB);
    }
    default:
        NVBLAST_ASSERT(0);
    }
    return support;
}




enum Status
{
    STATUS_NON_INTERSECT,
    STATUS_CONTACT,
    STATUS_DEGENERATE,
};

struct Output
{
    /// Get the normal to push apart in direction from A to B
    NV_FORCE_INLINE Vec3V getNormal() const { return V3Normalize(V3Sub(mClosestB, mClosestA)); }
    Vec3V mClosestA;                ///< Closest point on A
    Vec3V mClosestB;                ///< Closest point on B
    FloatV mDistSq;
};

struct ConvexV
{
    void calcExtent(const Vec3V& dir, float& minOut, float& maxOut) const
    {
        // Expand 
        const Vec4V x = Vec4V_From_FloatV(V3GetX(dir));
        const Vec4V y = Vec4V_From_FloatV(V3GetY(dir));
        const Vec4V z = Vec4V_From_FloatV(V3GetZ(dir));

        const Vec4V* src = mAovVertices;
        const Vec4V* end = src + mNumAovVertices * 3;

        // Do first step
        Vec4V max = V4MulAdd(x, src[0], V4MulAdd(y, src[1], V4Mul(z, src[2])));
        Vec4V min = max;
        src += 3;
        // Do the rest
        for (; src < end; src += 3)
        {
            const Vec4V dot = V4MulAdd(x, src[0], V4MulAdd(y, src[1], V4Mul(z, src[2])));
            max = V4Max(dot, max);
            min = V4Min(dot, min);
        }
        FStore(V4ExtractMax(max), &maxOut);
        FStore(V4ExtractMin(min), &minOut);
    }
    Vec3V calcSupport(const Vec3V& dir) const
    {
        // Expand 
        const Vec4V x = Vec4V_From_FloatV(V3GetX(dir));
        const Vec4V y = Vec4V_From_FloatV(V3GetY(dir));
        const Vec4V z = Vec4V_From_FloatV(V3GetZ(dir));

        NV_ALIGN(16, static const float index4const[]) = { 0.0f, 1.0f, 2.0f, 3.0f };
        Vec4V index4 = *(const Vec4V*)index4const;
        NV_ALIGN(16, static const float delta4const[]) = { 4.0f, 4.0f, 4.0f, 4.0f };
        const Vec4V delta4 = *(const Vec4V*)delta4const;

        const Vec4V* src = mAovVertices;
        const Vec4V* end = src + mNumAovVertices * 3;

        // Do first step
        Vec4V max = V4MulAdd(x, src[0], V4MulAdd(y, src[1], V4Mul(z, src[2])));
        Vec4V maxIndex = index4;
        index4 = V4Add(index4, delta4);
        src += 3;
        // Do the rest
        for (; src < end; src += 3)
        {
            const Vec4V dot = V4MulAdd(x, src[0], V4MulAdd(y, src[1], V4Mul(z, src[2])));
            const BoolV cmp = V4IsGrtr(dot, max);
            max = V4Max(dot, max);
            maxIndex = V4Sel(cmp, index4, maxIndex);
            index4 = V4Add(index4, delta4);
        }
        Vec4V horiMax = Vec4V_From_FloatV(V4ExtractMax(max));
        uint32_t mask = BGetBitMask(V4IsEq(horiMax, max));
        const uint32_t simdIndex = (0x12131210 >> (mask + mask)) & uint32_t(3);

        /// NOTE! Could be load hit store
        /// Would be better to have all simd. 
        NV_ALIGN(16, float f[4]);
        V4StoreA(maxIndex, f);
        uint32_t index = uint32_t(uint32_t(f[simdIndex]));

        const Vec4V* aovIndex = (mAovVertices + (index >> 2) * 3);
        const float* aovOffset = ((const float*)aovIndex) + (index & 3);

        return Vec3V_From_Vec4V(V4LoadXYZW(aovOffset[0], aovOffset[4], aovOffset[8], 1.0f));
    }

    const Vec4V* mAovVertices;          ///< Vertices storex x,x,x,x, y,y,y,y, z,z,z,z
    uint32_t mNumAovVertices;          ///< Number of groups of 4 of vertices
};

Status Collide(const Vec3V& initialDir, const ConvexV& convexA, const Mat34V& bToA, const ConvexV& convexB, Output& out)
{
    Vec3V Q[4];
    Vec3V A[4];
    Vec3V B[4];

    Mat33V aToB = M34Trnsps33(bToA);

    uint32_t size = 0;

    const Vec3V zeroV = V3Zero();
    const BoolV bTrue = BTTTT();

    //Vec3V v = V3UnitX();
    Vec3V v = V3Sel(FIsGrtr(V3Dot(initialDir, initialDir), FZero()), initialDir, V3UnitX());

    //const FloatV minMargin = zero;
    //const FloatV eps2 = FMul(minMargin, FLoad(0.01f));
    //FloatV eps2 = zero;
    FloatV eps2 = FLoad(1e-6f);
    const FloatV epsRel = FLoad(0.000225f);

    Vec3V closA(zeroV), closB(zeroV);
    FloatV sDist = FMax();
    FloatV minDist = sDist;
    Vec3V closAA = zeroV;
    Vec3V closBB = zeroV;

    BoolV bNotTerminated = bTrue;
    BoolV bCon = bTrue;

    do
    {
        minDist = sDist;
        closAA = closA;
        closBB = closB;

        uint32_t index = size++;
        NVBLAST_ASSERT(index < 4);

        const Vec3V supportA = convexA.calcSupport(V3Neg(v));
        const Vec3V supportB = M34MulV3(bToA, convexB.calcSupport(M33MulV3(aToB, v)));
        const Vec3V support = Vec3V_From_Vec4V(Vec4V_From_Vec3V(V3Sub(supportA, supportB)));

        A[index] = supportA;
        B[index] = supportB;
        Q[index] = support;

        const FloatV signDist = V3Dot(v, support);
        const FloatV tmp0 = FSub(sDist, signDist);
        if (FAllGrtr(FMul(epsRel, sDist), tmp0))
        {
            out.mClosestA = closA;
            out.mClosestB = closB;
            out.mDistSq = sDist;
            return STATUS_NON_INTERSECT;
        }

        //calculate the closest point between two convex hull
        v = doTesselation(Q, A, B, support, supportA, supportB, size, closA, closB);
        sDist = V3Dot(v, v);
        bCon = FIsGrtr(minDist, sDist);

        bNotTerminated = BAnd(FIsGrtr(sDist, eps2), bCon);
    } while (BAllEq(bNotTerminated, bTrue));

    out.mClosestA = V3Sel(bCon, closA, closAA);
    out.mClosestB = V3Sel(bCon, closB, closBB);
    out.mDistSq = FSel(bCon, sDist, minDist);
    return Status(BAllEq(bCon, bTrue) == 1 ? STATUS_CONTACT : STATUS_DEGENERATE);
}

static void _calcSeparation(const ConvexV& convexA, const nvidia::NvTransform& aToWorldIn, const Mat34V& bToA, ConvexV& convexB, const Vec3V& centroidAToB, Output& out, Separation& sep)
{
    
    Mat33V aToB = M34Trnsps33(bToA);
    Vec3V normalA = out.getNormal();
    FloatV vEpsilon = FLoad(1e-6f);
    if (BAllEqFFFF(FIsGrtr(out.mDistSq, vEpsilon)))
    {
        if (BAllEqTTTT(FIsGrtr(V3Dot(centroidAToB, centroidAToB), vEpsilon)))
        {
            normalA = V3Normalize(centroidAToB);
        }
        else
        {
            normalA = V3UnitX();
        }
    }

    convexA.calcExtent(normalA, sep.min0, sep.max0);
    Vec3V normalB = M33MulV3(aToB, normalA);
    convexB.calcExtent(normalB, sep.min1, sep.max1);

    {
        // Offset the min max taking into account transform
        // Distance of origin from B's space in As space in direction of the normal in As space should fix it...
        float fix;
        FStore(V3Dot(bToA.col3, normalA), &fix);
        sep.min1 += fix;
        sep.max1 += fix;
    }

    // Looks like it's the plane at the midpoint
    Vec3V center = V3Scale(V3Add(out.mClosestA, out.mClosestB), FLoad(0.5f));
    // Transform to world space
    Mat34V aToWorld;
    *(NvMat44*)&aToWorld = aToWorldIn;
    // Put the normal in world space
    Vec3V worldCenter = M34MulV3(aToWorld, center);
    Vec3V worldNormal = M34Mul33V3(aToWorld, normalA);

    FloatV dist = V3Dot(worldNormal, worldCenter);
    V3StoreU(worldNormal, sep.plane.n);
    FStore(dist, &sep.plane.d);
    sep.plane.d = -sep.plane.d;
}

static void _arrayVec3ToVec4(const NvVec3* src, Vec4V* dst, uint32_t num)
{
    const uint32_t num4 = num >> 2;
    for (uint32_t i = 0; i < num4; i++, dst += 3, src += 4)
    {
        Vec3V v0 = V3LoadU(&src[0].x);
        Vec3V v1 = V3LoadU(&src[1].x);
        Vec3V v2 = V3LoadU(&src[2].x);
        Vec3V v3 = V3LoadU(&src[3].x);
        // Transpose
        V4Transpose(v0, v1, v2, v3);
        // Save 
        dst[0] = v0;
        dst[1] = v1;
        dst[2] = v2;
    }
    const uint32_t remain = num & 3;
    if (remain)
    {
        Vec3V work[4];
        uint32_t i = 0;
        for (; i < remain; i++) work[i] = V3LoadU(&src[i].x);
        for (; i < 4; i++) work[i] = work[remain - 1];
        V4Transpose(work[0], work[1], work[2], work[3]);
        dst[0] = work[0];
        dst[1] = work[1];
        dst[2] = work[2];
    }
}


static void _arrayVec3ToVec4(const NvVec3* src, const Vec3V& scale, Vec4V* dst, uint32_t num)
{
    // If no scale - use the faster version
    if (V3AllEq(scale, V3One()))
    {
        return _arrayVec3ToVec4(src, dst, num);
    }

    const uint32_t num4 = num >> 2;
    for (uint32_t i = 0; i < num4; i++, dst += 3, src += 4)
    {
        Vec3V v0 = V3Mul(scale, V3LoadU(&src[0].x));
        Vec3V v1 = V3Mul(scale, V3LoadU(&src[1].x));
        Vec3V v2 = V3Mul(scale, V3LoadU(&src[2].x));
        Vec3V v3 = V3Mul(scale, V3LoadU(&src[3].x));
        // Transpose
        V4Transpose(v0, v1, v2, v3);
        // Save 
        dst[0] = v0;
        dst[1] = v1;
        dst[2] = v2;
    }
    const uint32_t remain = num & 3;
    if (remain)
    {
        Vec3V work[4];
        uint32_t i = 0;
        for (; i < remain; i++) work[i] = V3Mul(scale, V3LoadU(&src[i].x));
        for (; i < 4; i++) work[i] = work[remain - 1];
        V4Transpose(work[0], work[1], work[2], work[3]);
        dst[0] = work[0];
        dst[1] = work[1];
        dst[2] = work[2];
    }
}


// TODO: move this to a better long term home
// scope based helper struct to pick between stack and heap alloc based on the size of the request
struct ScopeMemoryAllocator {
public:
    ScopeMemoryAllocator() : mAlloc(nullptr) {};
    ~ScopeMemoryAllocator()
    {
        this->free();
    }

    void* alloc(size_t buffSize)
    {
        if (mAlloc == nullptr)
        {
            mAlloc = NVBLAST_ALLOC(buffSize);
            return mAlloc;
        }
        return nullptr;
    }

    void free()
    {
        if (mAlloc != nullptr)
        {
            NVBLAST_FREE(mAlloc);
            mAlloc = nullptr;
        }
    }

private:
    void* mAlloc;
};

#define STACK_ALLOC_LIMIT (100 * 1024)
#define ALLOCATE_TEMP_MEMORY(_out, buffSize)    \
    ScopeMemoryAllocator _out##Allocator;       \
    _out = (buffSize < STACK_ALLOC_LIMIT ? NvBlastAlloca(buffSize) : _out##Allocator.alloc(buffSize))


bool importerHullsInProximityApexFree(uint32_t hull0Count, const NvVec3* hull0, NvBounds3& hull0Bounds, const nvidia::NvTransform& localToWorldRT0In, const nvidia::NvVec3& scale0In,
    uint32_t hull1Count, const NvVec3* hull1, NvBounds3& hull1Bounds, const nvidia::NvTransform& localToWorldRT1In, const nvidia::NvVec3& scale1In,
    float maxDistance, Separation* separation)
{


    const uint32_t numVerts0 = static_cast<uint32_t>(hull0Count);
    const uint32_t numVerts1 = static_cast<uint32_t>(hull1Count);
    const uint32_t numAov0 = (numVerts0 + 3) >> 2;
    const uint32_t numAov1 = (numVerts1 + 3) >> 2;

    const uint32_t buffSize = (numAov0 + numAov1) * sizeof(Vec4V) * 3;
    void* buff = nullptr;
    ALLOCATE_TEMP_MEMORY(buff, buffSize);
    Vec4V* verts0 = (Vec4V*)buff;

    // Make sure it's aligned
    NVBLAST_ASSERT((size_t(verts0) & 0xf) == 0);

    Vec4V* verts1 = verts0 + (numAov0 * 3);

    const Vec3V scale0 = V3LoadU(&scale0In.x);
    const Vec3V scale1 = V3LoadU(&scale1In.x);
    std::vector<NvVec3> vert0(numVerts0);
    for (uint32_t i = 0; i < numVerts0; ++i)
    {
        vert0[i] = hull0[i];
    }
    std::vector<NvVec3> vert1(numVerts1);
    for (uint32_t i = 0; i < numVerts1; ++i)
    {
        vert1[i] = hull1[i];
    }

    _arrayVec3ToVec4(vert0.data(), scale0, verts0, numVerts0);
    _arrayVec3ToVec4(vert1.data(), scale1, verts1, numVerts1);

    const NvTransform trans1To0 = localToWorldRT0In.transformInv(localToWorldRT1In);

    // Load into simd mat
    Mat34V bToA;
    *(NvMat44*)&bToA = trans1To0;
    (*(NvMat44*)&bToA).column3.w = 0.0f; // AOS wants the 4th component of Vec3V to be 0 to work properly

    ConvexV convexA;
    ConvexV convexB;

    convexA.mNumAovVertices = numAov0;
    convexA.mAovVertices = verts0;

    convexB.mNumAovVertices = numAov1;
    convexB.mAovVertices = verts1;

    const nvidia::NvVec3 hullACenter = hull0Bounds.getCenter();
    const nvidia::NvVec3 hullBCenter = hull1Bounds.getCenter();
    const Vec3V centroidA = V3LoadU(&hullACenter.x);
    const Vec3V centroidB = M34MulV3(bToA, V3LoadU(&hullBCenter.x));

    // Take the origin of B in As space as the inital direction as it is 'the difference in transform origins B-A in A's space'
    // Should be a good first guess
    // Use centroid information
    const Vec3V initialDir = V3Sub(centroidB, centroidA);
    
    Output output;
    Status status = Collide(initialDir, convexA, bToA, convexB, output);

    if (status == STATUS_DEGENERATE)
    {
        // Calculate the tolerance from the extents
        const NvVec3 extents0 = hull0Bounds.getExtents();
        const NvVec3 extents1 = hull1Bounds.getExtents();

        const FloatV tolerance0 = V3ExtractMin(V3Mul(V3LoadU(&extents0.x), scale0));
        const FloatV tolerance1 = V3ExtractMin(V3Mul(V3LoadU(&extents1.x), scale1));

        const FloatV tolerance = FMul(FAdd(tolerance0, tolerance1), FLoad(0.01f));
        const FloatV sqTolerance = FMul(tolerance, tolerance);

        status = FAllGrtr(sqTolerance, output.mDistSq) ? STATUS_CONTACT : STATUS_NON_INTERSECT;
    }

    switch (status)
    {
    case STATUS_CONTACT:
    {
        if (separation)
        {
            _calcSeparation(convexA, localToWorldRT0In, bToA, convexB, initialDir, output, *separation);
        }
        return true;
    }
    default:
    case STATUS_NON_INTERSECT:
    {
        if (separation)
        {
            _calcSeparation(convexA, localToWorldRT0In, bToA, convexB, initialDir, output, *separation);
        }
        float val;
        FStore(output.mDistSq, &val);
        return val < (maxDistance * maxDistance);
    }
    }
}

} // namespace Blast
} // namespace Nv
