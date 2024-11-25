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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxMemory.h"
#include "foundation/PxMathUtils.h"
#include "DyConstraintPrep.h"
#include "DyCpuGpuArticulation.h"
#include "PxsRigidBody.h"
#include "DySolverConstraint1D.h"
#include "foundation/PxSort.h"
#include "DySolverConstraintDesc.h"
#include "PxcConstraintBlockStream.h"
#include "DyArticulationContactPrep.h"
#include "foundation/PxSIMDHelpers.h"
#include "DyArticulationUtils.h"
#include "DyAllocator.h"

namespace physx
{
namespace Dy
{
	// dsequeira:
	//
	// we can choose any linear combination of equality constraints and get the same solution
	// Hence we can orthogonalize the constraints using the inner product given by the
	// inverse mass matrix, so that when we use PGS, solving a constraint row for a joint 
	// don't disturb the solution of prior rows.
	//
	// We also eliminate the equality constraints from the hard inequality constraints - 
	// (essentially projecting the direction corresponding to the lagrange multiplier 
	// onto the equality constraint subspace) but 'til I've verified this generates 
	// exactly the same KKT/complementarity conditions, status is 'experimental'. 	
	//
	// since for equality constraints the resulting rows have the property that applying
	// an impulse along one row doesn't alter the projected velocity along another row, 
	// all equality constraints (plus one inequality constraint) can be processed in parallel
	// using SIMD
	//
	// Eliminating the inequality constraints from each other would require a solver change
	// and not give us any more parallelism, although we might get better convergence.

namespace
{
struct MassProps
{
	FloatV invMass0;  // the inverse mass of body0 after inverse mass scale was applied
	FloatV invMass1;  // the inverse mass of body1 after inverse mass scale was applied
	FloatV invInertiaScale0;
	FloatV invInertiaScale1;

	PX_FORCE_INLINE MassProps(const PxReal imass0, const PxReal imass1, const PxConstraintInvMassScale& ims) :
		invMass0(FLoad(imass0 * ims.linear0)),
		invMass1(FLoad(imass1 * ims.linear1)),
		invInertiaScale0(FLoad(ims.angular0)),
		invInertiaScale1(FLoad(ims.angular1))
	{}
};

PX_FORCE_INLINE PxReal innerProduct(const Px1DConstraint& row0, const Px1DConstraint& row1, 
								 const PxVec4& row0AngSqrtInvInertia0, const PxVec4& row0AngSqrtInvInertia1, 
								 const PxVec4& row1AngSqrtInvInertia0, const PxVec4& row1AngSqrtInvInertia1, const MassProps& m)
{
	const Vec3V l0 = V3Mul(V3Scale(V3LoadA(row0.linear0), m.invMass0), V3LoadA(row1.linear0));
	const Vec3V l1 = V3Mul(V3Scale(V3LoadA(row0.linear1), m.invMass1), V3LoadA(row1.linear1));
	const Vec4V r0ang0 = V4LoadA(&row0AngSqrtInvInertia0.x);
	const Vec4V r1ang0 = V4LoadA(&row1AngSqrtInvInertia0.x);
	const Vec4V r0ang1 = V4LoadA(&row0AngSqrtInvInertia1.x);
	const Vec4V r1ang1 = V4LoadA(&row1AngSqrtInvInertia1.x);

	const Vec3V i0 = V3ScaleAdd(V3Mul(Vec3V_From_Vec4V(r0ang0), Vec3V_From_Vec4V(r1ang0)), m.invInertiaScale0, l0);
	const Vec3V i1 = V3ScaleAdd(V3MulAdd(Vec3V_From_Vec4V(r0ang1), Vec3V_From_Vec4V(r1ang1), i0), m.invInertiaScale1, l1);
	PxF32 f;
	FStore(V3SumElems(i1), &f);
	return f;
}

// indexed rotation around axis, with sine and cosine of half-angle
PX_FORCE_INLINE PxQuat indexedRotation(PxU32 axis, PxReal s, PxReal c)
{
	PxQuat q(0,0,0,c);
	reinterpret_cast<PxReal*>(&q)[axis] = s;
	return q;
}

// PT: TODO: refactor with duplicate in FdMathUtils.cpp
PxQuat diagonalize(const PxMat33& m)	// jacobi rotation using quaternions 
{
	const PxU32 MAX_ITERS = 5;

	PxQuat q(PxIdentity);

	PxMat33 d;
	for(PxU32 i=0; i < MAX_ITERS;i++)
	{
		const PxMat33Padded axes(q);
		d = axes.getTranspose() * m * axes;

		const PxReal d0 = PxAbs(d[1][2]), d1 = PxAbs(d[0][2]), d2 = PxAbs(d[0][1]);
		const PxU32 a = PxU32(d0 > d1 && d0 > d2 ? 0 : d1 > d2 ? 1 : 2);	// rotation axis index, from largest off-diagonal element

		const PxU32 a1 = PxGetNextIndex3(a), a2 = PxGetNextIndex3(a1);											
		if(d[a1][a2] == 0.0f || PxAbs(d[a1][a1] - d[a2][a2]) > 2e6f * PxAbs(2.0f * d[a1][a2]))
			break;

		const PxReal w = (d[a1][a1] - d[a2][a2]) / (2.0f * d[a1][a2]);	// cot(2 * phi), where phi is the rotation angle
		const PxReal absw = PxAbs(w);

		PxQuat r;
		if(absw > 1000)
			r = indexedRotation(a, 1.0f / (4.0f * w), 1.0f);	// h will be very close to 1, so use small angle approx instead
		else
		{
  			const PxReal t = 1.0f / (absw + PxSqrt(w * w + 1.0f));	// absolute value of tan phi
			const PxReal h = 1.0f / PxSqrt(t * t + 1.0f);			// absolute value of cos phi

			PX_ASSERT(h != 1);	// |w|<1000 guarantees this with typical IEEE754 machine eps (approx 6e-8)
			r = indexedRotation(a, PxSqrt((1.0f - h) / 2.0f) * PxSign(w), PxSqrt((1.0f + h) / 2.0f));
		}
	
		q = (q * r).getNormalized();
	}

	return q;
}

PX_FORCE_INLINE void rescale(const Mat33V& m, PxVec3& a0, PxVec3& a1, PxVec3& a2)
{
	const Vec3V va0 = V3LoadU(a0);
	const Vec3V va1 = V3LoadU(a1);
	const Vec3V va2 = V3LoadU(a2);

	const Vec3V b0 = V3ScaleAdd(va0, V3GetX(m.col0), V3ScaleAdd(va1, V3GetY(m.col0), V3Scale(va2, V3GetZ(m.col0))));
	const Vec3V b1 = V3ScaleAdd(va0, V3GetX(m.col1), V3ScaleAdd(va1, V3GetY(m.col1), V3Scale(va2, V3GetZ(m.col1))));
	const Vec3V b2 = V3ScaleAdd(va0, V3GetX(m.col2), V3ScaleAdd(va1, V3GetY(m.col2), V3Scale(va2, V3GetZ(m.col2))));

	V3StoreU(b0, a0);
	V3StoreU(b1, a1);
	V3StoreU(b2, a2);
}

PX_FORCE_INLINE void rescale4(const Mat33V& m, PxReal* a0, PxReal* a1, PxReal* a2)
{
	const Vec4V va0 = V4LoadA(a0);
	const Vec4V va1 = V4LoadA(a1);
	const Vec4V va2 = V4LoadA(a2);

	const Vec4V b0 = V4ScaleAdd(va0, V3GetX(m.col0), V4ScaleAdd(va1, V3GetY(m.col0), V4Scale(va2, V3GetZ(m.col0))));
	const Vec4V b1 = V4ScaleAdd(va0, V3GetX(m.col1), V4ScaleAdd(va1, V3GetY(m.col1), V4Scale(va2, V3GetZ(m.col1))));
	const Vec4V b2 = V4ScaleAdd(va0, V3GetX(m.col2), V4ScaleAdd(va1, V3GetY(m.col2), V4Scale(va2, V3GetZ(m.col2))));

	V4StoreA(b0, a0);
	V4StoreA(b1, a1);
	V4StoreA(b2, a2);
}

void diagonalize(Px1DConstraint** row,
				 PxVec4* angSqrtInvInertia0,
				 PxVec4* angSqrtInvInertia1,
				 const MassProps& m)
{
	const PxReal a00 = innerProduct(*row[0], *row[0], angSqrtInvInertia0[0], angSqrtInvInertia1[0], angSqrtInvInertia0[0], angSqrtInvInertia1[0], m);
	const PxReal a01 = innerProduct(*row[0], *row[1], angSqrtInvInertia0[0], angSqrtInvInertia1[0], angSqrtInvInertia0[1], angSqrtInvInertia1[1], m);
	const PxReal a02 = innerProduct(*row[0], *row[2], angSqrtInvInertia0[0], angSqrtInvInertia1[0], angSqrtInvInertia0[2], angSqrtInvInertia1[2], m);
	const PxReal a11 = innerProduct(*row[1], *row[1], angSqrtInvInertia0[1], angSqrtInvInertia1[1], angSqrtInvInertia0[1], angSqrtInvInertia1[1], m);
	const PxReal a12 = innerProduct(*row[1], *row[2], angSqrtInvInertia0[1], angSqrtInvInertia1[1], angSqrtInvInertia0[2], angSqrtInvInertia1[2], m);
	const PxReal a22 = innerProduct(*row[2], *row[2], angSqrtInvInertia0[2], angSqrtInvInertia1[2], angSqrtInvInertia0[2], angSqrtInvInertia1[2], m);

	const PxMat33 a(PxVec3(a00, a01, a02),
					PxVec3(a01, a11, a12),
					PxVec3(a02, a12, a22));

	const PxQuat q = diagonalize(a);

	const PxMat33 n(-q);

	const Mat33V mn(V3LoadU(n.column0), V3LoadU(n.column1), V3LoadU(n.column2));

	//KS - We treat as a Vec4V so that we get geometricError rescaled for free along with linear0
	rescale4(mn, &row[0]->linear0.x, &row[1]->linear0.x, &row[2]->linear0.x);
	rescale(mn, row[0]->linear1, row[1]->linear1, row[2]->linear1);
	//KS - We treat as a PxVec4 so that we get velocityTarget rescaled for free 
	rescale4(mn, &row[0]->angular0.x, &row[1]->angular0.x, &row[2]->angular0.x);
	rescale(mn, row[0]->angular1, row[1]->angular1, row[2]->angular1);
	rescale4(mn, &angSqrtInvInertia0[0].x, &angSqrtInvInertia0[1].x, &angSqrtInvInertia0[2].x);
	rescale4(mn, &angSqrtInvInertia1[0].x, &angSqrtInvInertia1[1].x, &angSqrtInvInertia1[2].x);
}

//
// A 1D constraint between two bodies (b0, b1) acts on specific linear and angular velocity
// directions of these two bodies. Let the constrained linear velocity direction for body b0
// be l0 and the constrained angular velocity direction be a0. Likewise, let l1 and a1 be
// the corresponding constrained velocity directions for body b1.
// 
// Let the constraint Jacobian J be the 1x12 vector that combines the 3x1 vectors l0, a0, l1, a1
// J = | l0^T, a0^T, l1^T, a1^T |
// 
// Let vl0, va0, vl1, va1 be the 3x1 linear/angular velocites of two bodies
// and v be the 12x1 combination of those:
// 
//     | vl0 |
// v = | va0 |
//     | vl1 |
//     | va1 |
//
// The constraint projected velocity scalar is then:
// projV = J * v
// 
// Let M be the 12x12 mass matrix (with scalar masses m0, m1 and 3x3 inertias I0, I1)
// 
// | m0                            |
// |    m0                         |
// |       m0                      |
// |         |    |                |
// |         | I0 |                |
// |         |    |                |
// |                m1             |
// |                   m1          |
// |                      m1       |
// |                        |    | |
// |                        | I1 | |
// |                        |    | |
// 
// Let p be the impulse scalar that results from solving the 1D constraint given
// projV, geometric error etc.
// Turning this impulse p to a 12x1 delta velocity vector dv:
// 
// dv = M^-1 * (J^T * p) =  M^-1 * J^T * p
// 
// Now to consider the case of multiple 1D constraints J0, J1, J2, ... operating
// on the same body pair.
// 
// Let K be the matrix holding these constraints as rows
// 
//     |  J0  |
// K = |  J1  |
//     |  J2  |
//     |  ... |
// 
// Applying these constraints:
//
// |                 |                | p0       |
// | dv0 dv1 dv2 ... | = M^-1 * K^T * |    p1    |
// |                 |                |       p2 |
//
// Let MK = (M^-1 * K^T)^T = K * M^-1  (M^-1 is symmetric). The transpose
// is only used here to talk about constraint rows instead of columns (since
// that expression seems to be used more commonly here).
// 
// dvMatrix = MK^T * pMatrix
// 
// The rows of MK define how an impulse affects the constrained velocity directions
// of the two bodies. Ideally, the different constraint rows operate on independent
// parts of the velocity such that constraints don't step on each others toe
// (constraint A making the situation better with respect to its own constrained
// velocity directions but worse for directions of constraint B and vice versa).
// A formal way to specify this goal is to try to have the rows of MK be orthogonal
// to each other, that is, to orthogonalize the MK matrix. This will eliminate
// any action of a constraint in directions that have been touched by previous
// constraints already. This re-configuration of constraint rows does not work in
// general but for hard equality constraints (no spring, targetVelocity=0,
// min/maxImpulse unlimited), changing the constraint rows to make them orthogonal
// should not change the solution of the constraint problem. As an example, one
// might consider a joint with two 1D constraints that lock linear movement in the
// xy-plane (those are hard equality constraints). It's fine to choose different
// constraint directions from the ones provided, assuming the new directions are
// still in the xy-plane and that the geometric errors get patched up accordingly.
// 
// \param[in,out] row Pointers to the constraints to orthogonalize. The members
//                    linear0/1, angular0/1, geometricError and velocityTarget will
//                    get changed potentially
// \param[in,out] angSqrtInvInertia0 body b0 angular velocity directions of the
//                                   constraints provided in parameter row but
//                                   multiplied by the square root ot the inverse
//                                   inertia tensor of body b0.
//                                   I0^(-1/2) * angularDirection0, ...
//                                   Will be replaced by the orthogonalized vectors.
//                                   Note: the fourth component of the vectors serves
//                                   no purpose in this method.
// \param[in,out] angSqrtInvInertia1 Same as previous parameter but for body b1.
// \param[in] rowCount Number of entries in row, angSqrtInvInertia0, angSqrtInvInertia1
// \param[in] eqRowCount Number of entries in row that represent equality constraints.
//                       The method expects the entries in row to be sorted by equality
//                       constraints first, followed by inequality constraints. The
//                       latter get orthogonalized relative to the equality constraints
//                       but not relative to the other inequality constraints.
// \param[in] m	Some mass properties of the two bodies b0, b1.
//
void orthogonalize(Px1DConstraint** row,
				   PxVec4* angSqrtInvInertia0,
				   PxVec4* angSqrtInvInertia1,
				   PxU32 rowCount,
				   PxU32 eqRowCount,
				   const MassProps &m)
{
	PX_ASSERT(eqRowCount<=6);

	const FloatV zero = FZero();

	Vec3V lin1m[6], ang1m[6], lin1[6], ang1[6];	
	Vec4V lin0m[6], ang0m[6];			// must have 0 in the W-field
	Vec4V lin0AndG[6], ang0AndT[6];

	for(PxU32 i=0;i<rowCount;i++)
	{
		Vec4V l0AndG = V4LoadA(&row[i]->linear0.x);		// linear0 and geometric error
		Vec4V a0AndT = V4LoadA(&row[i]->angular0.x);	// angular0 and velocity target

		Vec3V l1 = Vec3V_From_Vec4V(V4LoadA(&row[i]->linear1.x));
		Vec3V a1 = Vec3V_From_Vec4V(V4LoadA(&row[i]->angular1.x));

		Vec4V angSqrtL0 = V4LoadA(&angSqrtInvInertia0[i].x);
		Vec4V angSqrtL1 = V4LoadA(&angSqrtInvInertia1[i].x);

		const PxU32 eliminationRows = PxMin<PxU32>(i, eqRowCount);
		for(PxU32 j=0;j<eliminationRows;j++)
		{
			//
			// Gram-Schmidt algorithm to get orthogonal vectors. A set of vectors
			// v0, v1, v2..., can be turned into orthogonal vectors u0, u1, u2, ...
			// as follows:
			// 
			// u0 = v0
			// u1 = v1 - proj_u0(v1)
			// u2 = v2 - proj_u0(v2) - proj_u1(v2)
			// ...
			// 
			// proj_u(v) denotes the resulting vector when vector v gets
			// projected onto the normalized vector u.
			// 
			//    __ v
			//     /|
			//    /
			//   /
			//  /
			// ----->---------------->
			//  proj_u(v)            u
			//
			// Let <v,u> be the dot/inner product of the two vectors v and u.
			// 
			// proj_u(v) = <v,normalize(u)> * normalize(u)
			//           = <v,u/|u|> * (u/|u|) = <v,u> / (|u|*|u|)  *  u
			//           = <v,u> / <u,u>  *  u
			// 
			// The implementation here maps as follows:
			//
			// u = [orthoLinear0, orthoAngular0, orthoLinear1, orthoAngular1]
			// v = [row[]->linear0, row[]->angular0, row[]->linear1, row[]->angular1]
			//
			// Since the solver is using momocity, orthogonality should not be achieved for rows
			// M^-1 * u but for rows uM = M^(-1/2) * u (with M^(-1/2) being the square root of the
			// inverse mass matrix). Following the described orthogonalization procedure to turn
			// v1m into u1m that is orthogonal to u0m:
			// 
			// u1M           = v1M - proj_u0M(v1M)
			// M^(-1/2) * u1 = M^(-1/2) * v1  -  <v1M,u0M>/<u0M,u0M> * M^(-1/2) * u0
			// 
			// Since M^(-1/2) is multiplied on the left and right hand side, this can be transformed to:
			// 
			// u1 = v1  -  <v1M,u0M>/<u0M,u0M> * u0
			// 
			// For the computation of <v1M,u0M>/<u0M,u0M>, the following shall be considered:
			//
			// <vM,uM>
			// = <M^(-1/2) * v, M^(-1/2) * u>
			// = (M^(-1/2) * v)^T * (M^(-1/2) * u)  (v and u being seen as 12x1 vectors here)
			// = v^T * M^(-1/2)^T * M^(-1/2) * u
			// = v^T * M^-1 * u   (M^(-1/2) is a symmetric matrix, thus transposing has no effect)
			// = <v, M^-1 * u>
			// 
			// Applying this:
			// 
			// <v1M,u0M>/<u0M,u0M> = <v1, M^-1 * u0> / <u0, M^-1 * u0>
			//
			// The code uses:
			// 
			// v1m_ = [v1Lin0, I0^(-1/2) * v1Ang0, v1Lin1, I1^(-1/2) * v1Ang1]
			// u0m_ = [(1/m0) * u0Lin0, I0^(-1/2) * u0Ang0, (1/m1) * u0Lin1, I1^(-1/2) * u0Ang1]
			// u0m* = u0m_ / <u0M,u0M>  (see variables named lin0m, ang0m, lin1m, ang1m in the code)
			// 
			// And then does:
			// 
			// <v1m_, u0m*> = <v1m_, u0m_> / <u0M,u0M> = <v, M^-1 * u> / <u0M,u0M> = <v1M,u0M>/<u0M,u0M>
			// (see variable named t in the code)
			// 
			// note: u0, u1, ... get computed for equality constraints. Inequality constraints do not generate new
			//       "base" vectors. Let's say u0, u1 are from equality constraints, then for inequality constraints
			//       u2, u3:
			// 
			//       u2 = v2 - proj_u0(v2) - proj_u1(v2)
			//       u3 = v3 - proj_u0(v3) - proj_u1(v3)
			//
			//       in other words: the inequality constraints will be orthogonal to the equality constraints but not
			//       to other inequality constraints.
			//

			const Vec3V s0 = V3MulAdd(l1, lin1m[j], Vec3V_From_Vec4V_WUndefined(V4Mul(l0AndG, lin0m[j])));
			const Vec3V s1 = V3MulAdd(Vec3V_From_Vec4V_WUndefined(angSqrtL1), ang1m[j], Vec3V_From_Vec4V_WUndefined(V4Mul(angSqrtL0, ang0m[j])));
			const FloatV t = V3SumElems(V3Add(s0, s1));

			l0AndG = V4NegScaleSub(lin0AndG[j], t, l0AndG);  // note: this can reduce the error term by the amount covered by the orthogonal base vectors
			a0AndT = V4NegScaleSub(ang0AndT[j], t, a0AndT);  // note: for equality and inequality constraints, target velocity is expected to be 0
			l1 = V3NegScaleSub(lin1[j], t, l1);
			a1 = V3NegScaleSub(ang1[j], t, a1);
			angSqrtL0 = V4NegScaleSub(V4LoadA(&angSqrtInvInertia0[j].x), t, angSqrtL0);
			angSqrtL1 = V4NegScaleSub(V4LoadA(&angSqrtInvInertia1[j].x), t, angSqrtL1);
			// note: angSqrtL1 is equivalent to I1^(-1/2) * a1 (and same goes for angSqrtL0)
		}

		V4StoreA(l0AndG, &row[i]->linear0.x);
		V4StoreA(a0AndT, &row[i]->angular0.x);
		V3StoreA(l1, row[i]->linear1);
		V3StoreA(a1, row[i]->angular1);
		V4StoreA(angSqrtL0, &angSqrtInvInertia0[i].x);
		V4StoreA(angSqrtL1, &angSqrtInvInertia1[i].x);

		if(i<eqRowCount)
		{
			lin0AndG[i] = l0AndG;	
			ang0AndT[i] = a0AndT;
			lin1[i] = l1;	
			ang1[i] = a1;

			//
			// compute the base vector used for orthogonalization (see comments further above).
			//
			
			const Vec3V l0 = Vec3V_From_Vec4V(l0AndG);

			const Vec3V l0m = V3Scale(l0, m.invMass0);  // note that the invMass values used here have invMassScale applied already
			const Vec3V l1m = V3Scale(l1, m.invMass1);
			const Vec4V a0m = V4Scale(angSqrtL0, m.invInertiaScale0);
			const Vec4V a1m = V4Scale(angSqrtL1, m.invInertiaScale1);

			const Vec3V s0 = V3MulAdd(l0, l0m, V3Mul(l1, l1m));
			const Vec4V s1 = V4MulAdd(a0m, angSqrtL0, V4Mul(a1m, angSqrtL1));
			const FloatV s = V3SumElems(V3Add(s0, Vec3V_From_Vec4V_WUndefined(s1)));
			const FloatV a = FSel(FIsGrtr(s, zero), FRecip(s), zero);	// with mass scaling, it's possible for the inner product of a row to be zero

			lin0m[i] = V4Scale(V4ClearW(Vec4V_From_Vec3V(l0m)), a);	
			ang0m[i] = V4Scale(V4ClearW(a0m), a);
			lin1m[i] = V3Scale(l1m, a);
			ang1m[i] = V3Scale(Vec3V_From_Vec4V_WUndefined(a1m), a);
		}
	}
}
}

// PT: make sure that there's at least a PxU32 after angular0/angular1 in the Px1DConstraint structure (for safe SIMD reads)
// Note that the code was V4LoadAding these before anyway so it must be safe already.
// PT: removed for now because some compilers didn't like it
//PX_COMPILE_TIME_ASSERT((sizeof(Px1DConstraint) - PX_OFFSET_OF_RT(Px1DConstraint, angular0)) >= (sizeof(PxVec3) + sizeof(PxU32)));
//PX_COMPILE_TIME_ASSERT((sizeof(Px1DConstraint) - PX_OFFSET_OF_RT(Px1DConstraint, angular1)) >= (sizeof(PxVec3) + sizeof(PxU32)));

// PT: TODO: move somewhere else
PX_FORCE_INLINE Vec3V M33MulV4(const Mat33V& a, const Vec4V b)
{
	const FloatV x = V4GetX(b);
	const FloatV y = V4GetY(b);
	const FloatV z = V4GetZ(b);
	const Vec3V v0 = V3Scale(a.col0, x);
	const Vec3V v1 = V3Scale(a.col1, y);
	const Vec3V v2 = V3Scale(a.col2, z);
	const Vec3V v0PlusV1 = V3Add(v0, v1);
	return V3Add(v0PlusV1, v2);
}

void preprocessRows(Px1DConstraint** sorted, 
					Px1DConstraint* rows,
					PxVec4* angSqrtInvInertia0,
					PxVec4* angSqrtInvInertia1,
					PxU32 rowCount,
					const PxMat33& sqrtInvInertia0F32,
					const PxMat33& sqrtInvInertia1F32,
					const PxReal invMass0,
					const PxReal invMass1,
					const PxConstraintInvMassScale& ims,
					bool disablePreprocessing,
					bool diagonalizeDrive)
{
	// j is maxed at 12, typically around 7, so insertion sort is fine
	for(PxU32 i=0; i<rowCount; i++)
	{
		Px1DConstraint* r = rows+i;
		
		PxU32 j = i;
		for(;j>0 && r->solveHint < sorted[j-1]->solveHint; j--)
			sorted[j] = sorted[j-1];

		sorted[j] = r;
	}

	for(PxU32 i=0;i<rowCount-1;i++)
		PX_ASSERT(sorted[i]->solveHint <= sorted[i+1]->solveHint);

	// PT: it is always safe to use "V3LoadU_SafeReadW" on the two first columns of a PxMat33. However in this case the passed matrices
	// come from PxSolverBodyData::sqrtInvInertia (PGS) or PxTGSSolverBodyTxInertia::sqrtInvInertia (TGS). It is currently unsafe to use
	// V3LoadU_SafeReadW in the TGS case (the matrix is the last element of the structure). So we keep V3LoadU here for now. For PGS we
	// have a compile-time-assert (see PxSolverBodyData struct) to ensure safe reads on sqrtInvInertia.
	// Note that because we only use this in M33MulV4 below, the ClearW calls could also be skipped.
	const Mat33V sqrtInvInertia0 = Mat33V(
		V3LoadU_SafeReadW(sqrtInvInertia0F32.column0),
		V3LoadU_SafeReadW(sqrtInvInertia0F32.column1),
		V3LoadU(sqrtInvInertia0F32.column2));

	const Mat33V sqrtInvInertia1 = Mat33V(
		V3LoadU_SafeReadW(sqrtInvInertia1F32.column0),
		V3LoadU_SafeReadW(sqrtInvInertia1F32.column1),
		V3LoadU(sqrtInvInertia1F32.column2));

	PX_ASSERT(((uintptr_t(angSqrtInvInertia0)) & 0xF) == 0);
	PX_ASSERT(((uintptr_t(angSqrtInvInertia1)) & 0xF) == 0);

	for(PxU32 i=0; i<rowCount; ++i)
	{
		// PT: new version is 10 instructions smaller
		//const Vec3V angDelta0_ = M33MulV3(sqrtInvInertia0, V3LoadU(sorted[i]->angular0));
		//const Vec3V angDelta1_ = M33MulV3(sqrtInvInertia1, V3LoadU(sorted[i]->angular1));
		const Vec3V angDelta0 = M33MulV4(sqrtInvInertia0, V4LoadA(&sorted[i]->angular0.x));
		const Vec3V angDelta1 = M33MulV4(sqrtInvInertia1, V4LoadA(&sorted[i]->angular1.x));
		V4StoreA(Vec4V_From_Vec3V(angDelta0), &angSqrtInvInertia0[i].x);
		V4StoreA(Vec4V_From_Vec3V(angDelta1), &angSqrtInvInertia1[i].x);
	}

	if(disablePreprocessing)
		return;

	MassProps m(invMass0, invMass1, ims);
	for(PxU32 i=0;i<rowCount;)
	{
		const PxU32 groupMajorId = PxU32(sorted[i]->solveHint>>8), start = i++;
		while(i<rowCount && PxU32(sorted[i]->solveHint>>8) == groupMajorId)
			i++;

		if(groupMajorId == 4 || (groupMajorId == 8))
		{
			//
			// PGS:
			// - make all equality constraints orthogonal to each other
			// - make all inequality constraints orthogonal to all equality constraints
			// This assumes only PxConstraintSolveHint::eEQUALITY and ::eINEQUALITY is used.
			//
			// TGS:
			// - make all linear equality constraints orthogonal to each other
			// - make all linear inequality constraints orthogonal to all linear equality constraints
			// - make all angular equality constraints orthogonal to each other
			// - make all angular inequality constraints orthogonal to all angular equality constraints
			// This is achieved by internally turning PxConstraintSolveHint::eEQUALITY and ::eINEQUALITY into
			// ::eROTATIONAL_EQUALITY and ::eROTATIONAL_INEQUALITY for angular constraints.
			//

			PxU32 bCount = start;		// count of bilateral constraints 
			for(; bCount<i && (sorted[bCount]->solveHint&255)==0; bCount++)
				;
			orthogonalize(sorted+start, angSqrtInvInertia0+start, angSqrtInvInertia1+start, i-start, bCount-start, m);
		}

		if(groupMajorId == 1 && diagonalizeDrive)
		{			
			PxU32 slerp = start;		// count of bilateral constraints 
			for(; slerp<i && (sorted[slerp]->solveHint&255)!=2; slerp++)
				;
			if(slerp+3 == i)
				diagonalize(sorted+slerp, angSqrtInvInertia0+slerp, angSqrtInvInertia1+slerp, m);

			PX_ASSERT(i-start==3);
			diagonalize(sorted+start, angSqrtInvInertia0+start, angSqrtInvInertia1+start, m);
		}
	}
}

PxU32 ConstraintHelper::setupSolverConstraint(
PxSolverConstraintPrepDesc& prepDesc,
PxConstraintAllocator& allocator,
PxReal simDt, PxReal recipSimDt)
{
	if (prepDesc.numRows == 0)
	{
		prepDesc.desc->constraint = NULL;
		prepDesc.desc->writeBack = NULL;
		prepDesc.desc->constraintLengthOver16 = 0;
		return 0;
	}

	PxSolverConstraintDesc& desc = *prepDesc.desc;

	const bool isExtended = (desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY)
		|| (desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY);

	const PxU32 stride = isExtended ? sizeof(SolverConstraint1DExt) : sizeof(SolverConstraint1D);
	const PxU32 constraintLength = sizeof(SolverConstraint1DHeader) + stride * prepDesc.numRows;
	
	//KS - +16 is for the constraint progress counter, which needs to be the last element in the constraint (so that we
	//know SPU DMAs have completed)
	PxU8* ptr = allocator.reserveConstraintData(constraintLength + 16u);
	if(!checkConstraintDataPtr<true>(ptr))
		return 0;

	desc.constraint = ptr;

	setConstraintLength(desc,constraintLength);

	desc.writeBack = prepDesc.writeback;

	PxMemSet(desc.constraint, 0, constraintLength);

	SolverConstraint1DHeader* header = reinterpret_cast<SolverConstraint1DHeader*>(desc.constraint);
	PxU8* constraints = desc.constraint + sizeof(SolverConstraint1DHeader);
	init(*header, PxTo8(prepDesc.numRows), isExtended, prepDesc.invMassScales);
	header->body0WorldOffset = prepDesc.body0WorldOffset;
	header->linBreakImpulse = prepDesc.linBreakForce * simDt;
	header->angBreakImpulse = prepDesc.angBreakForce * simDt;
	header->breakable = PxU8((prepDesc.linBreakForce != PX_MAX_F32) || (prepDesc.angBreakForce != PX_MAX_F32));
	header->invMass0D0 = prepDesc.data0->invMass * prepDesc.invMassScales.linear0;
	header->invMass1D1 = prepDesc.data1->invMass * prepDesc.invMassScales.linear1;

	PX_ALIGN(16, PxVec4) angSqrtInvInertia0[MAX_CONSTRAINT_ROWS];
	PX_ALIGN(16, PxVec4) angSqrtInvInertia1[MAX_CONSTRAINT_ROWS];
	
	Px1DConstraint* sorted[MAX_CONSTRAINT_ROWS];

	preprocessRows(sorted, prepDesc.rows, angSqrtInvInertia0, angSqrtInvInertia1, prepDesc.numRows, 
		prepDesc.data0->sqrtInvInertia, prepDesc.data1->sqrtInvInertia, prepDesc.data0->invMass, prepDesc.data1->invMass, 
		prepDesc.invMassScales, isExtended || prepDesc.disablePreprocessing, prepDesc.improvedSlerp);

	const PxReal erp = 1.0f;

	PxU32 outCount = 0;

	const SolverExtBody eb0(reinterpret_cast<const void*>(prepDesc.body0), prepDesc.data0, desc.linkIndexA);
	const SolverExtBody eb1(reinterpret_cast<const void*>(prepDesc.body1), prepDesc.data1, desc.linkIndexB);

	PxReal cfm = 0.f;
	if (isExtended)
	{
		cfm = PxMax(eb0.getCFM(), eb1.getCFM());
	}

	for (PxU32 i = 0; i<prepDesc.numRows; i++)
	{
		PxPrefetchLine(constraints, 128);
		SolverConstraint1D& s = *reinterpret_cast<SolverConstraint1D *>(constraints);
		Px1DConstraint& c = *sorted[i];

		PxReal minImpulse, maxImpulse;
		computeMinMaxImpulseOrForceAsImpulse(
			c.minImpulse, c.maxImpulse,
			c.flags & Px1DConstraintFlag::eHAS_DRIVE_LIMIT, prepDesc.driveLimitsAreForces, simDt,
			minImpulse, maxImpulse);

		PxReal unitResponse;
		PxReal jointSpeedForRestitutionBounce = 0.0f;
		PxReal initJointSpeed = 0.0f;

		const PxReal minResponseThreshold = prepDesc.minResponseThreshold;

		if(!isExtended)
		{
			//The theoretical formulation of the Jacobian J has 4 terms {linear0, angular0, linear1, angular1}
			//s.lin0 and s.lin1 match J.linear0 and J.linear1.
			//We compute s.ang0 and s.ang1 but these *must not* be confused with the angular terms of the theoretical Jacobian.
			//s.ang0 and s.ang1 are part of the momocity system that computes deltas to the angular motion that are neither
			//angular momentum nor angular velocity.
			//s.ang0 = I0^(-1/2) * J.angular0 with I0 denoting the inertia of body0 in the world frame.
			//s.ang1 = I1^(-1/2) * J.angular1 with I1 denoting the inertia of body1 in the world frame.
			//We then compute the unit response r = J * M^-1 * JTranspose with M denoting the mass matrix.
			//r = (1/m0)*|J.linear0|^2 + (1/m1)*|J.linear1|^2 + J.angular0 * I0^-1 * J.angular0 +  J.angular1 * I1^-1 * J.angular1
			//We can write out the term [J.angular0 * I0^-1 * J.angular0] in a different way:
			//J.angular0 * I0^-1 * J.angular0 = [J.angular0 * I0^(-1/2)] dot  [I0^(-1/2) * J.angular0]
			//Noting that s.ang0 =  J.angular0 * I0^(-1/2) and the equivalent expression for body 1, we have the following:
			//r = 	(1/m0)*|s.lin0|^2 + (1/m1)*|s.lin1|^2 + |s.ang0|^2 + |s.ang1|^2 
			//Init vel is computed using the standard Jacobian method because at this stage we have linear and angular velocities.
			//The code that resolves the constraints instead accumulates delta linear velocities and delta angular momocities compatible with 
			//s.lin0/lin1 and s.ang0/ang1. Right now, though, we have only linear and angular velocities compatible with the theoretical
			//Jacobian form:
			//initVel = J.linear0.dot(linVel0) + J.angular0.dot(angvel0) - J.linear1.dot(linVel1) - J.angular1.dot(angvel1) 
			init(s, c.linear0, c.linear1, PxVec3(angSqrtInvInertia0[i].x, angSqrtInvInertia0[i].y, angSqrtInvInertia0[i].z),
				PxVec3(angSqrtInvInertia1[i].x, angSqrtInvInertia1[i].y, angSqrtInvInertia1[i].z), minImpulse, maxImpulse);
			s.ang0Writeback = c.angular0;
			const PxReal resp0 = s.lin0.magnitudeSquared() * prepDesc.data0->invMass * prepDesc.invMassScales.linear0 + s.ang0.magnitudeSquared() * prepDesc.invMassScales.angular0;
			const PxReal resp1 = s.lin1.magnitudeSquared() * prepDesc.data1->invMass * prepDesc.invMassScales.linear1 + s.ang1.magnitudeSquared() * prepDesc.invMassScales.angular1;
			unitResponse = resp0 + resp1;
			initJointSpeed = jointSpeedForRestitutionBounce = prepDesc.data0->projectVelocity(c.linear0, c.angular0) - prepDesc.data1->projectVelocity(c.linear1, c.angular1);
		}
		else
		{
			//this is articulation/deformable volume
			init(s, c.linear0, c.linear1, c.angular0, c.angular1, minImpulse, maxImpulse);
			SolverConstraint1DExt& e = static_cast<SolverConstraint1DExt&>(s);

			const Cm::SpatialVector resp0 = createImpulseResponseVector(e.lin0, e.ang0, eb0);
			const Cm::SpatialVector resp1 = createImpulseResponseVector(-e.lin1, -e.ang1, eb1);
			unitResponse = getImpulseResponse(eb0, resp0, unsimdRef(e.deltaVA), prepDesc.invMassScales.linear0, prepDesc.invMassScales.angular0,
				eb1, resp1, unsimdRef(e.deltaVB), prepDesc.invMassScales.linear1, prepDesc.invMassScales.angular1, false);

			//Add CFM term!

			if(unitResponse <= DY_ARTICULATION_MIN_RESPONSE)
				continue;
			unitResponse += cfm;
			
			s.ang0Writeback = c.angular0;
			s.lin0 = resp0.linear;
			s.ang0 = resp0.angular;
			s.lin1 = -resp1.linear;
			s.ang1 = -resp1.angular;
			PxReal vel0, vel1;
			const bool b0IsRigidDynamic = (eb0.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY);
			const bool b1IsRigidDynamic = (eb1.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY);
			if(needsNormalVel(c) || b0IsRigidDynamic || b1IsRigidDynamic)
			{
				vel0 = eb0.projectVelocity(c.linear0, c.angular0);
				vel1 = eb1.projectVelocity(c.linear1, c.angular1);

				Dy::computeJointSpeedPGS(vel0, b0IsRigidDynamic, vel1, b1IsRigidDynamic, jointSpeedForRestitutionBounce, initJointSpeed);
			}

			//minResponseThreshold = PxMax(minResponseThreshold, DY_ARTICULATION_MIN_RESPONSE);
		}

		const PxReal recipUnitResponse = computeRecipUnitResponse(unitResponse, minResponseThreshold);
		s.setSolverConstants(
			compute1dConstraintSolverConstantsPGS(
			c.flags, 
			c.mods.spring.stiffness, c.mods.spring.damping, 
			c.mods.bounce.restitution, c.mods.bounce.velocityThreshold, 
			c.geometricError, c.velocityTarget,
			jointSpeedForRestitutionBounce, initJointSpeed, 
			unitResponse, recipUnitResponse, 
			erp, 
			simDt, recipSimDt));

		if(c.flags & Px1DConstraintFlag::eOUTPUT_FORCE)
			s.setOutputForceFlag(true);

		outCount++;

		constraints += stride;
	}

	//Reassign count to the header because we may have skipped some rows if they were degenerate
	header->count = PxU8(outCount);
	return prepDesc.numRows;
}

PxU32 SetupSolverConstraint(SolverConstraintShaderPrepDesc& shaderDesc,
	PxSolverConstraintPrepDesc& prepDesc,
	PxConstraintAllocator& allocator,
	PxReal dt, PxReal invdt)
{
	// LL shouldn't see broken constraints
	
	PX_ASSERT(!(reinterpret_cast<ConstraintWriteback*>(prepDesc.writeback)->isBroken()));

	setConstraintLength(*prepDesc.desc, 0);

	if (!shaderDesc.solverPrep)
		return 0;

	//PxU32 numAxisConstraints = 0;

	Px1DConstraint rows[MAX_CONSTRAINT_ROWS];
	setupConstraintRows(rows, MAX_CONSTRAINT_ROWS);

	prepDesc.invMassScales.linear0 = prepDesc.invMassScales.linear1 = prepDesc.invMassScales.angular0 = prepDesc.invMassScales.angular1 = 1.0f;
	prepDesc.body0WorldOffset = PxVec3(0.0f);

	PxVec3p unused_ra, unused_rb;

	//TAG::solverprepcall
	prepDesc.numRows = prepDesc.disableConstraint ? 0 : (*shaderDesc.solverPrep)(rows,
		prepDesc.body0WorldOffset,
		MAX_CONSTRAINT_ROWS,
		prepDesc.invMassScales,
		shaderDesc.constantBlock,
		prepDesc.bodyFrame0, prepDesc.bodyFrame1, prepDesc.extendedLimits, unused_ra, unused_rb);

	prepDesc.rows = rows;

	return ConstraintHelper::setupSolverConstraint(prepDesc, allocator, dt, invdt);
}

}

}
