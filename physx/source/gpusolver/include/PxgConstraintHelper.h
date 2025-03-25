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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_CONSTRAINT_HELPER_H
#define PXG_CONSTRAINT_HELPER_H

#include "PxgD6JointLimit.h"
#include "foundation/PxQuat.h"
#include "foundation/PxMathUtils.h"
#include "CmConeLimitHelper.h"

// PT: TODO: refactor/share remaining code. One reason for the duplication is that the CPU code uses
// SIMD here and there, while the GPU code doesn't. But we could still merge the two eventually.

namespace physx
{
	// PT: TODO: this is a duplicate of the one in Extensions, but less robust?
	PX_CUDA_CALLABLE PX_INLINE void computeJacobianAxes(PxVec3 row[3], const PxQuat& qa, const PxQuat& qb)
	{
		// Compute jacobian matrix for (qa* qb)  [[* means conjugate in this expr]]
		// d/dt (qa* qb) = 1/2 L(qa*) R(qb) (omega_b - omega_a)
		// result is L(qa*) R(qb), where L(q) and R(q) are left/right q multiply matrix

		PxReal wa = qa.w, wb = qb.w;
		const PxVec3 va(qa.x,qa.y,qa.z), vb(qb.x,qb.y,qb.z);

		const PxVec3 c = vb*wa + va*wb;
		const PxReal d = wa*wb - va.dot(vb);

		row[0] = (va * vb.x + vb * va.x + PxVec3(d,     c.z, -c.y)) * 0.5f;
		row[1] = (va * vb.y + vb * va.y + PxVec3(-c.z,  d,    c.x)) * 0.5f;
		row[2] = (va * vb.z + vb * va.z + PxVec3(c.y,   -c.x,   d)) * 0.5f;
	}

	PX_INLINE PX_CUDA_CALLABLE void computeJointFrames(PxTransform& cA2w, PxTransform& cB2w, const PxgJointData& data, const PxTransform& bA2w, const PxTransform& bB2w)
	{
		PX_ASSERT(bA2w.isValid() && bB2w.isValid());

		cA2w = bA2w.transform(data.c2b[0]);
		cB2w = bB2w.transform(data.c2b[1]);

		PX_ASSERT(cA2w.isValid() && cB2w.isValid());
	}

	class PxgConstraintHelper
	{
		PxVec3 mRa, mRb;

	public:
		PX_CUDA_CALLABLE PxgConstraintHelper(/*Px1DConstraint* c,*/ const PxVec3& ra, const PxVec3& rb)
			: /*mConstraints(c), mCurrent(c),*/ mRa(ra), mRb(rb)	{}

		PX_CUDA_CALLABLE PxgConstraintHelper(
			PxTransform& cA2w, PxTransform& cB2w,
			const PxgJointData& data, const PxTransform& bA2w, const PxTransform& bB2w)
		{
			computeJointFrames(cA2w, cB2w, data, bA2w, bB2w);

			mRa = cB2w.p - bA2w.p;
			mRb = cB2w.p - bB2w.p;
		}

		// hard linear & angular
		PX_FORCE_INLINE void linearHard(Px1DConstraint* c, const PxVec3& axis, PxReal posErr)
		{
			linear(c, axis, posErr, PxConstraintSolveHint::eEQUALITY);
			c->flags |= Px1DConstraintFlag::eOUTPUT_FORCE;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void angularHard(Px1DConstraint* c, const PxVec3& axis, PxReal posErr)
		{
			angular(c, axis, posErr, PxConstraintSolveHint::eEQUALITY);
			c->flags |= Px1DConstraintFlag::eOUTPUT_FORCE;
		}

		// limited linear & angular
		PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 linearLimit(Px1DConstraint* c, PxU32 currentIndex, const PxVec3& axis, PxReal ordinate, PxReal limitValue, const PxgJointLimitParameters& limit)
		{
			if(!limit.isSoft() || ordinate > limitValue)
			{
				Px1DConstraint* cConstraint = &c[currentIndex++];
				linear(cConstraint, axis,limitValue - ordinate, PxConstraintSolveHint::eNONE);
				addLimit(cConstraint ,limit);
			}
			return currentIndex;
		}

		PX_FORCE_INLINE PxU32 angularLimit(Px1DConstraint* c, PxU32 currentIndex, const PxVec3& axis, PxReal ordinate, PxReal limitValue, PxReal pad, const PxgJointLimitParameters& limit)
		{
			if(limit.isSoft())
				pad = 0;

			if(ordinate + pad > limitValue)
			{
				Px1DConstraint* cConstraint = &c[currentIndex++];
				angular(cConstraint, axis,limitValue - ordinate, PxConstraintSolveHint::eNONE);
				addLimit(cConstraint,limit);
			}
			return currentIndex;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void angularLimit(Px1DConstraint* c, const PxVec3& axis, PxReal error, const PxgJointLimitParameters& limit)const
		{
			angular(c, axis,error, PxConstraintSolveHint::eNONE);
			addLimit(c,limit);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 anglePair(Px1DConstraint* c, PxU32 currentIndex, PxReal angle, PxReal lower, PxReal upper, const PxVec3& axis, const PxgJointLimitParameters& limit)const
		{
			PX_ASSERT(lower<upper);
			const bool softLimit = limit.isSoft();

			if (!softLimit || angle < lower)
				angularLimit(&c[currentIndex++], -axis, -(lower - angle), limit);
			if (!softLimit || angle > upper)
				angularLimit(&c[currentIndex++], axis, (upper - angle), limit);

			return currentIndex;
		}

		// driven linear & angular

		PX_CUDA_CALLABLE PX_FORCE_INLINE void linear(Px1DConstraint* c, const PxVec3& axis, PxReal velTarget, PxReal error, const PxgD6JointDrive& drive)const
		{
			linear(c, axis,error,PxConstraintSolveHint::eNONE);
			addDrive(c,velTarget,drive);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void angular(Px1DConstraint* c, const PxVec3& axis, PxReal velTarget, PxReal error, const PxgD6JointDrive& drive, PxConstraintSolveHint::Enum hint = PxConstraintSolveHint::eNONE)const
		{
			angular(c, axis,error,hint);
			addDrive(c,velTarget,drive);
		}

		//PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getCount() { return PxU32(mCurrent - mConstraints); }

		PX_CUDA_CALLABLE PxU32 prepareLockedAxes(Px1DConstraint* c, PxU32 currentIndex, const PxQuat& qA, const PxQuat& qB, const PxVec3& cB2cAp, PxU32 lin, PxU32 ang,
			PxVec3& raOut)
		{
			//Px1DConstraint* current = mCurrent;
			//const PxU32 startIndex = currentIndex;

			PxVec3 errorVector(0.f);

			PxVec3 ra = mRa;

			if(lin)
			{
				PxMat33 axes(qA);

				if (lin & 1) errorVector -= axes.column0 * cB2cAp.x;
				if (lin & 2) errorVector -= axes.column1 * cB2cAp.y;
				if (lin & 4) errorVector -= axes.column2 * cB2cAp.z;

				ra += errorVector;

				if(lin&1) linear(&c[currentIndex++], axes[0], ra, mRb, -cB2cAp[0], PxConstraintSolveHint::eEQUALITY, Px1DConstraintFlag::eOUTPUT_FORCE);
				if(lin&2) linear(&c[currentIndex++], axes[1], ra, mRb, -cB2cAp[1], PxConstraintSolveHint::eEQUALITY, Px1DConstraintFlag::eOUTPUT_FORCE);
				if(lin&4) linear(&c[currentIndex++], axes[2], ra, mRb, -cB2cAp[2], PxConstraintSolveHint::eEQUALITY, Px1DConstraintFlag::eOUTPUT_FORCE);
			}

			if (ang)
			{
				PxQuat qB2qA = qA.getConjugate() * qB;
				/*if (qB2qA.w<0)
					qB2qA = -qB2qA;*/

				PxVec3 row[3];
				computeJacobianAxes(row, qA, qB);
				PxVec3 imp = qB2qA.getImaginaryPart();
				if (ang & 1) angular(&c[currentIndex++], row[0], -imp.x, PxConstraintSolveHint::eEQUALITY, Px1DConstraintFlag::eOUTPUT_FORCE);
				if (ang & 2) angular(&c[currentIndex++], row[1], -imp.y, PxConstraintSolveHint::eEQUALITY, Px1DConstraintFlag::eOUTPUT_FORCE);
				if (ang & 4) angular(&c[currentIndex++], row[2], -imp.z, PxConstraintSolveHint::eEQUALITY, Px1DConstraintFlag::eOUTPUT_FORCE);
			}

			raOut = ra;

			return currentIndex;
		}

	private:
		PX_CUDA_CALLABLE PX_FORCE_INLINE void linear(Px1DConstraint* c, const PxVec3& axis, PxReal posErr, PxConstraintSolveHint::Enum hint)const
		{
			c->solveHint		= PxU16(hint);
			c->linear0 = axis;					c->angular0	= mRa.cross(axis);
			c->linear1 = axis;					c->angular1 = mRb.cross(axis);
			PX_ASSERT(c->linear0.isFinite());
			PX_ASSERT(c->linear1.isFinite());
			PX_ASSERT(c->angular0.isFinite());
			PX_ASSERT(c->angular1.isFinite());

			c->geometricError	= posErr;

			c->flags = 0;
			c->minImpulse = -PX_MAX_REAL;
			c->maxImpulse = PX_MAX_REAL;
			c->mods.spring.damping = 0.f;
			c->mods.spring.stiffness = 0.f;
			c->velocityTarget =0.f;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void linear(Px1DConstraint* c, const PxVec3& axis, const PxVec3& ra, const PxVec3& rb, PxReal posErr, PxConstraintSolveHint::Enum hint,
			PxU32 flags = 0)const
		{
			c->solveHint = PxU16(hint);
			c->linear0 = axis;					c->angular0 = ra.cross(axis);
			c->linear1 = axis;					c->angular1 = rb.cross(axis);
			PX_ASSERT(c->linear0.isFinite());
			PX_ASSERT(c->linear1.isFinite());
			PX_ASSERT(c->angular0.isFinite());
			PX_ASSERT(c->angular1.isFinite());

			c->geometricError = posErr;

			c->flags = flags;
			c->minImpulse = -PX_MAX_REAL;
			c->maxImpulse = PX_MAX_REAL;
			c->mods.spring.damping = 0.f;
			c->mods.spring.stiffness = 0.f;
			c->velocityTarget = 0.f;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void angular(Px1DConstraint* c, const PxVec3& axis, PxReal posErr, PxConstraintSolveHint::Enum hint,
			PxU32 flags = 0)const
		{
			c->solveHint		= PxU16(hint);
			c->linear0 = PxVec3(0);		c->angular0			= axis;
			c->linear1 = PxVec3(0);		c->angular1			= axis;
			c->geometricError	= posErr;

			c->flags = flags | Px1DConstraintFlag::eANGULAR_CONSTRAINT;
			c->minImpulse = -PX_MAX_REAL;
			c->maxImpulse = PX_MAX_REAL;
			c->mods.spring.damping = 0.f;
			c->mods.spring.stiffness = 0.f;
			c->velocityTarget = 0.f;
		}

		PX_CUDA_CALLABLE void addLimit(Px1DConstraint* c, const PxgJointLimitParameters& limit)const
		{
			PxU16 flags = PxU16(c->flags | Px1DConstraintFlag::eOUTPUT_FORCE);

			if(limit.isSoft())
			{
				flags |= Px1DConstraintFlag::eSPRING;
				c->mods.spring.stiffness = limit.stiffness;
				c->mods.spring.damping = limit.damping;
			}
			else
			{
				c->solveHint = PxConstraintSolveHint::eINEQUALITY;
				c->mods.bounce.restitution = limit.restitution;
				c->mods.bounce.velocityThreshold = limit.bounceThreshold;
				if(c->geometricError>0)
					flags |= Px1DConstraintFlag::eKEEPBIAS;
				if(limit.restitution>0)
					flags |= Px1DConstraintFlag::eRESTITUTION;
			}

			c->flags = flags;
			c->minImpulse = 0;
		}

		PX_CUDA_CALLABLE void addDrive(Px1DConstraint* c, PxReal velTarget, const PxgD6JointDrive& drive)const
		{
			c->velocityTarget = velTarget;

			PxU16 flags = PxU16(c->flags | Px1DConstraintFlag::eSPRING | Px1DConstraintFlag::eHAS_DRIVE_LIMIT);

			if(drive.flags & PxgD6JointDriveFlag::eACCELERATION)
				flags |= Px1DConstraintFlag::eACCELERATION_SPRING;

			if (drive.flags & PxgD6JointDriveFlag::eOUTPUT_FORCE)
				flags |= Px1DConstraintFlag::eOUTPUT_FORCE;

			c->flags = flags;
			c->mods.spring.stiffness = drive.stiffness;
			c->mods.spring.damping = drive.damping;
				
			c->minImpulse = -drive.forceLimit;
			c->maxImpulse = drive.forceLimit;

			//PX_ASSERT(c->linear0.isFinite());
			//PX_ASSERT(c->angular0.isFinite());
		}
	};
}

#endif