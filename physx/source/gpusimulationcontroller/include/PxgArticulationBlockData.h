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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_ARTICULATION_BLOCK_DATA_H
#define PXG_ARTICULATION_BLOCK_DATA_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "PxSpatialMatrix.h"
#include "DyCpuGpuArticulation.h"
#include "cutil_math.h"

namespace physx
{
	//We load/store to Cm::SpatialMatrix rather than PxSpatialMatrix.
	//This exploits the symmetry of the spatial matrix to avoid storing/loading the bottom-right,
	//which is the transpose of the top-left
	struct PxgSpatialMatrixBlock
	{
		float4	columns[7][32];
	};

	//We load/store to a PxSpatialMatrix. This has a full 6x6 matrix as there is no
	//symmetry as described in the above struct
	struct PxgSpatialMatrix6x6Block
	{
		float4	columns[9][32];
	};

	struct PxgMat33Block
	{
		float4	mCol0[32];
		float4	mCol1[32];
		float	mCol2[32];
	};

	struct PxgSpatialVectorBlock
	{
		float4 mTopxyz_bx[32];
		float2 mbyz[32];
	};

	struct PxgSpatialTransformBlock
	{
		float4 q[32];
		float4 p[32];
	};


	PX_FORCE_INLINE PX_CUDA_CALLABLE void loadSpatialMatrix(const PxgSpatialMatrixBlock& block, const PxU32 threadIndexInWarp, Dy::SpatialMatrix& mat)
	{
		float4 val = block.columns[0][threadIndexInWarp];
		mat.topLeft[0][0] = val.x; mat.topLeft[0][1] = val.y; mat.topLeft[0][2] = val.z; mat.topLeft[1][0] = val.w;
		val = block.columns[1][threadIndexInWarp];
		mat.topLeft[1][1] = val.x; mat.topLeft[1][2] = val.y; mat.topLeft[2][0] = val.z; mat.topLeft[2][1] = val.w;
		val = block.columns[2][threadIndexInWarp];
		mat.topLeft[2][2] = val.x; mat.topRight[0][0] = val.y; mat.topRight[0][1] = val.z; mat.topRight[0][2] = val.w;
		val = block.columns[3][threadIndexInWarp];
		mat.topRight[1][0] = val.x; mat.topRight[1][1] = val.y; mat.topRight[1][2] = val.z; mat.topRight[2][0] = val.w;
		val = block.columns[4][threadIndexInWarp];
		mat.topRight[2][1] = val.x; mat.topRight[2][2] = val.y; mat.bottomLeft[0][0] = val.z; mat.bottomLeft[0][1] = val.w;
		val = block.columns[5][threadIndexInWarp];
		mat.bottomLeft[0][2] = val.x; mat.bottomLeft[1][0] = val.y; mat.bottomLeft[1][1] = val.z; mat.bottomLeft[1][2] = val.w;
		val = block.columns[6][threadIndexInWarp];
		mat.bottomLeft[2][0] = val.x; mat.bottomLeft[2][1] = val.y; mat.bottomLeft[2][2] = val.z;
	}

	//This loads a 7-vec matrix in which the bottomRight is equal to the transpose of the top left
	PX_FORCE_INLINE PX_CUDA_CALLABLE void loadSpatialMatrix(const PxgSpatialMatrixBlock& block, const PxU32 threadIndexInWarp, PxSpatialMatrix& mat)
	{
		float4 val = block.columns[0][threadIndexInWarp];
		mat.column[0][0] = val.x; mat.column[0][1] = val.y; mat.column[0][2] = val.z; mat.column[1][0] = val.w;
		mat.column[3][3] = val.x; mat.column[4][3] = val.y; mat.column[5][3] = val.z; mat.column[3][4] = val.w;
		val = block.columns[1][threadIndexInWarp];
		mat.column[1][1] = val.x; mat.column[1][2] = val.y; mat.column[2][0] = val.z; mat.column[2][1] = val.w;
		mat.column[4][4] = val.x; mat.column[5][4] = val.y; mat.column[3][5] = val.z; mat.column[4][5] = val.w;
		val = block.columns[2][threadIndexInWarp];
		mat.column[2][2] = val.x; mat.column[0][3] = val.y; mat.column[0][4] = val.z; mat.column[0][5] = val.w;
		mat.column[5][5] = val.x;
		val = block.columns[3][threadIndexInWarp];
		mat.column[1][3] = val.x; mat.column[1][4] = val.y; mat.column[1][5] = val.z; mat.column[2][3] = val.w;
		val = block.columns[4][threadIndexInWarp];
		mat.column[2][4] = val.x; mat.column[2][5] = val.y; mat.column[3][0] = val.z; mat.column[3][1] = val.w;
		val = block.columns[5][threadIndexInWarp];
		mat.column[3][2] = val.x; mat.column[4][0] = val.y; mat.column[4][1] = val.z; mat.column[4][2] = val.w;
		val = block.columns[6][threadIndexInWarp];
		mat.column[5][0] = val.x; mat.column[5][1] = val.y; mat.column[5][2] = val.z;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void storeSpatialMatrix(PxgSpatialMatrixBlock& block, const PxU32 threadIndexInWarp, const Dy::SpatialMatrix& mat)
	{
		block.columns[0][threadIndexInWarp] = make_float4(mat.topLeft[0][0], mat.topLeft[0][1], mat.topLeft[0][2], mat.topLeft[1][0]);
		block.columns[1][threadIndexInWarp] = make_float4(mat.topLeft[1][1], mat.topLeft[1][2], mat.topLeft[2][0], mat.topLeft[2][1]);
		block.columns[2][threadIndexInWarp] = make_float4(mat.topLeft[2][2], mat.topRight[0][0], mat.topRight[0][1], mat.topRight[0][2]);
		block.columns[3][threadIndexInWarp] = make_float4(mat.topRight[1][0], mat.topRight[1][1], mat.topRight[1][2], mat.topRight[2][0]);
		block.columns[4][threadIndexInWarp] = make_float4(mat.topRight[2][1], mat.topRight[2][2], mat.bottomLeft[0][0], mat.bottomLeft[0][1]);
		block.columns[5][threadIndexInWarp] = make_float4(mat.bottomLeft[0][2], mat.bottomLeft[1][0], mat.bottomLeft[1][1], mat.bottomLeft[1][2]);
		block.columns[6][threadIndexInWarp] = make_float4(mat.bottomLeft[2][0], mat.bottomLeft[2][1], mat.bottomLeft[2][2], 0.f);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void loadSpatialMatrix(const PxgSpatialMatrix6x6Block& block, const PxU32 threadIndexInWarp, PxSpatialMatrix& mat)
	{
		float4 val = block.columns[0][threadIndexInWarp];
		mat.column[0][0] = val.x; mat.column[0][1] = val.y; mat.column[0][2] = val.z; mat.column[0][3] = val.w;
		val = block.columns[1][threadIndexInWarp];
		mat.column[0][4] = val.x; mat.column[0][5] = val.y; mat.column[1][0] = val.z; mat.column[1][1] = val.w;
		val = block.columns[2][threadIndexInWarp];
		mat.column[1][2] = val.x; mat.column[1][3] = val.y; mat.column[1][4] = val.z; mat.column[1][5] = val.w;
		val = block.columns[3][threadIndexInWarp];
		mat.column[2][0] = val.x; mat.column[2][1] = val.y; mat.column[2][2] = val.z; mat.column[2][3] = val.w;
		val = block.columns[4][threadIndexInWarp];
		mat.column[2][4] = val.x; mat.column[2][5] = val.y; mat.column[3][0] = val.z; mat.column[3][1] = val.w;
		val = block.columns[5][threadIndexInWarp];
		mat.column[3][2] = val.x; mat.column[3][3] = val.y; mat.column[3][4] = val.z; mat.column[3][5] = val.w;
		val = block.columns[6][threadIndexInWarp];
		mat.column[4][0] = val.x; mat.column[4][1] = val.y; mat.column[4][2] = val.z; mat.column[4][3] = val.w;
		val = block.columns[7][threadIndexInWarp];
		mat.column[4][4] = val.x; mat.column[4][5] = val.y; mat.column[5][0] = val.z; mat.column[5][1] = val.w;
		val = block.columns[8][threadIndexInWarp];
		mat.column[5][2] = val.x; mat.column[5][3] = val.y; mat.column[5][4] = val.z; mat.column[5][5] = val.w;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void storeSpatialMatrix(PxgSpatialMatrix6x6Block& block, const PxU32 threadIndexInWarp, const PxSpatialMatrix& mat)
	{
		block.columns[0][threadIndexInWarp] = make_float4(mat.column[0][0], mat.column[0][1], mat.column[0][2], mat.column[0][3]);
		block.columns[1][threadIndexInWarp] = make_float4(mat.column[0][4], mat.column[0][5], mat.column[1][0], mat.column[1][1]);
		block.columns[2][threadIndexInWarp] = make_float4(mat.column[1][2], mat.column[1][3], mat.column[1][4], mat.column[1][5]);
		block.columns[3][threadIndexInWarp] = make_float4(mat.column[2][0], mat.column[2][1], mat.column[2][2], mat.column[2][3]);
		block.columns[4][threadIndexInWarp] = make_float4(mat.column[2][4], mat.column[2][5], mat.column[3][0], mat.column[3][1]);
		block.columns[5][threadIndexInWarp] = make_float4(mat.column[3][2], mat.column[3][3], mat.column[3][4], mat.column[3][5]);
		block.columns[6][threadIndexInWarp] = make_float4(mat.column[4][0], mat.column[4][1], mat.column[4][2], mat.column[4][3]);
		block.columns[7][threadIndexInWarp] = make_float4(mat.column[4][4], mat.column[4][5], mat.column[5][0], mat.column[5][1]);
		block.columns[8][threadIndexInWarp] = make_float4(mat.column[5][2], mat.column[5][3], mat.column[5][4], mat.column[5][5]);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void zeroSpatialMatrix(PxgSpatialMatrix6x6Block& block, const PxU32 threadIndexInWarp)
	{
		block.columns[0][threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 0.f);
		block.columns[1][threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 0.f);
		block.columns[2][threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 0.f);
		block.columns[3][threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 0.f);
		block.columns[4][threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 0.f);
		block.columns[5][threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 0.f);
		block.columns[6][threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 0.f);
		block.columns[7][threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 0.f);
		block.columns[8][threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 0.f);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE Cm::SpatialVectorF loadSpatialVectorF(const PxgSpatialVectorBlock& block, const PxU32 threadIndexInWarp)
	{
		float4 top = block.mTopxyz_bx[threadIndexInWarp];
		float2 bottom = block.mbyz[threadIndexInWarp];

		return Cm::SpatialVectorF(PxVec3(top.x, top.y, top.z), PxVec3(top.w, bottom.x, bottom.y));
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE Cm::UnAlignedSpatialVector loadSpatialVector(const PxgSpatialVectorBlock& block, const PxU32 threadIndexInWarp)
	{
		float4 top = block.mTopxyz_bx[threadIndexInWarp];
		float2 bottom = block.mbyz[threadIndexInWarp];

		return Cm::UnAlignedSpatialVector(PxVec3(top.x, top.y, top.z), PxVec3(top.w, bottom.x, bottom.y));
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void storeSpatialVector(PxgSpatialVectorBlock& block, const Cm::UnAlignedSpatialVector& v, const PxU32 threadIndexInWarp)
	{
		block.mTopxyz_bx[threadIndexInWarp] = make_float4(v.top.x, v.top.y, v.top.z, v.bottom.x);
		block.mbyz[threadIndexInWarp] = make_float2(v.bottom.y, v.bottom.z);

		/*Pxstcg(&block.mTopxyz_bx[threadIndexInWarp], make_float4(v.top.x, v.top.y, v.top.z, v.bottom.x));
		Pxstcg(&block.mbyz[threadIndexInWarp], make_float2(v.bottom.y, v.bottom.z));*/
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void addSpatialVector(PxgSpatialVectorBlock& block, const Cm::UnAlignedSpatialVector& v, const PxU32 threadIndexInWarp)
	{
		block.mTopxyz_bx[threadIndexInWarp] += make_float4(v.top.x, v.top.y, v.top.z, v.bottom.x);
		block.mbyz[threadIndexInWarp] += make_float2(v.bottom.y, v.bottom.z);

		/*Pxstcg(&block.mTopxyz_bx[threadIndexInWarp], make_float4(v.top.x, v.top.y, v.top.z, v.bottom.x));
		Pxstcg(&block.mbyz[threadIndexInWarp], make_float2(v.bottom.y, v.bottom.z));*/
	}

#if PX_CUDA_COMPILER
	PX_FORCE_INLINE __device__ void atomicAddSpatialVector(PxgSpatialVectorBlock& block, const Cm::UnAlignedSpatialVector& v, const PxU32 threadIndexInWarp)
	{
#if __CUDA_ARCH__ >= 200
		atomicAdd(&block.mTopxyz_bx[threadIndexInWarp].x, v.top.x);
		atomicAdd(&block.mTopxyz_bx[threadIndexInWarp].y, v.top.y);
		atomicAdd(&block.mTopxyz_bx[threadIndexInWarp].z, v.top.z);

		atomicAdd(&block.mTopxyz_bx[threadIndexInWarp].w, v.bottom.x);
		atomicAdd(&block.mbyz[threadIndexInWarp].x, v.bottom.y);
		atomicAdd(&block.mbyz[threadIndexInWarp].y, v.bottom.z);
#else
		PX_UNUSED(block);
		PX_UNUSED(v);
		PX_UNUSED(threadIndexInWarp);
#endif

		/*Pxstcg(&block.mTopxyz_bx[threadIndexInWarp], make_float4(v.top.x, v.top.y, v.top.z, v.bottom.x));
		Pxstcg(&block.mbyz[threadIndexInWarp], make_float2(v.bottom.y, v.bottom.z));*/
	}
#endif

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxTransform loadSpatialTransform(const PxgSpatialTransformBlock& block, const PxU32 threadIndexInWarp)
	{
		const float4 q = block.q[threadIndexInWarp];
		const float4 p = block.p[threadIndexInWarp];

		return PxTransform(PxVec3(p.x, p.y, p.z), PxQuat(q.x, q.y, q.z, q.w));
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void storeSpatialTransform(PxgSpatialTransformBlock& block, const PxU32 threadIndexInWarp, const PxTransform& transform)
	{
		block.q[threadIndexInWarp] = make_float4(transform.q.x, transform.q.y, transform.q.z, transform.q.w);
		block.p[threadIndexInWarp] = make_float4(transform.p.x, transform.p.y, transform.p.z, 0.f);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxQuat loadQuat(const float4* f, const PxU32 threadIndexInWarp)
	{
		float4 q = f[threadIndexInWarp];
		return PxQuat(q.x, q.y, q.z, q.w);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxVec3 loadPxVec3(const float4* f, const PxU32 threadIndexInWarp)
	{
		float4 v = f[threadIndexInWarp];
		return PxVec3(v.x, v.y, v.z);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void loadPxMat33(const PxgMat33Block& block, const PxU32 threadIndexInWarp, PxMat33& mat)
	{
		float4 v0 = block.mCol0[threadIndexInWarp];
		float4 v1 = block.mCol1[threadIndexInWarp];
		float v2 = block.mCol2[threadIndexInWarp];

		mat.column0.x = v0.x; mat.column0.y = v0.y; mat.column0.z = v0.z;
		mat.column1.x = v0.w; mat.column1.y = v1.x; mat.column1.z = v1.y;
		mat.column2.x = v1.z; mat.column2.y = v1.w; mat.column2.z = v2;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void storePxMat33(PxgMat33Block& block, const PxU32 threadIndexInWarp, const PxMat33& mat)
	{
		block.mCol0[threadIndexInWarp] = make_float4(mat.column0.x, mat.column0.y, mat.column0.z, mat.column1.x);
		block.mCol1[threadIndexInWarp] = make_float4(mat.column1.y, mat.column1.z, mat.column2.x, mat.column2.y);
		block.mCol2[threadIndexInWarp] = mat.column2.z;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE Cm::UnAlignedSpatialVector loadSpatialVector(const Cm::UnAlignedSpatialVector* PX_RESTRICT vector)
	{
		size_t ptr = size_t(vector);
		if (ptr & 0xf)
		{
			float2* top = reinterpret_cast<float2*>(ptr);
			float2 val = *top;
			float4 val2 = *(reinterpret_cast<float4*>(top+1));

			return Cm::UnAlignedSpatialVector(PxVec3(val.x, val.y, val2.x), PxVec3(val2.y, val2.z, val2.w));
		}
		else
		{
			float4* top = reinterpret_cast<float4*>(ptr);
			float4 val = *top;
			float2 val2 = *(reinterpret_cast<float2*>(top + 1));

			return Cm::UnAlignedSpatialVector(PxVec3(val.x, val.y, val.z), PxVec3(val.w, val2.x, val2.y));
		}
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE void storeSpatialVector(Cm::UnAlignedSpatialVector* PX_RESTRICT vector,
		const Cm::UnAlignedSpatialVector& src)
	{
		size_t ptr = size_t(vector);
		if (ptr & 0xf)
		{
			float2* top = reinterpret_cast<float2*>(ptr);
			float4* bottom = (reinterpret_cast<float4*>(top + 1));

			*top = make_float2(src.top.x, src.top.y);
			*bottom = make_float4(src.top.z, src.bottom.x, src.bottom.y, src.bottom.z);
		}
		else
		{
			float4* top = reinterpret_cast<float4*>(ptr);
			float2* bottom = (reinterpret_cast<float2*>(top + 1));

			*top = make_float4(src.top.x, src.top.y, src.top.z, src.bottom.x);
			*bottom = make_float2(src.bottom.y, src.bottom.z);
		}
	}


	struct PxgArticulationBitFieldStackData
	{
		PxU64 bitField[32];
	};

	typedef PxgArticulationBitFieldStackData PxgArticulationBitFieldData;

	struct PxgArticulationTraversalStackData
	{
		PxU32 indices[32];
		PxgSpatialVectorBlock impulseStack;
		PxgSpatialVectorBlock deltaVStack;
	};

	struct PxgArticulationBlockLinkData
	{
		PxgSpatialMatrixBlock			mSpatialArticulatedInertia;
		PxgSpatialMatrix6x6Block		mSpatialResponseMatrix;
		PxgMat33Block					mIsolatedInertia;
		PxReal							mMass[32];

		//KS - these are questionable - we might want to store velocity/deltaVelocity etc. in a non-SOA format
		//for the constraint solver. To be revisited later!!!!
		PxgSpatialVectorBlock			mMotionVelocity;
		PxgSpatialVectorBlock			mPosMotionVelocity;
		PxgSpatialVectorBlock			mDeltaMotion;
		PxgSpatialVectorBlock			mScratchImpulse; // These are non-propagated impulses. They may persist between kernels and are properly processed and then reset in averageLinkImpulsesAndPropagate. Should be reset to zero after propagation.
		PxgSpatialVectorBlock			mScratchDeltaV; // Temp! Used only for propagating velocities around the tree structure within a kernel! Left in an undefined state after use.

		PxgSpatialVectorBlock			mSolverSpatialDeltaVel;
		PxgSpatialVectorBlock			mSolverSpatialImpulse;
		PxgSpatialVectorBlock			mSolverSpatialInternalConstraintImpulse;

		PxgSpatialVectorBlock			mZAVector;
		PxgSpatialVectorBlock			mZAIntVector;
		PxgSpatialVectorBlock			mCoriolis;
		PxgSpatialVectorBlock			mConstraintForces;

		PxgSpatialVectorBlock			mMotionAcceleration;
		PxgSpatialVectorBlock			mMotionAccelerationInternal;
		PxgSpatialVectorBlock			mBiasForce;

		PxReal							mDeltaScale[32];

		float							mRw_x[32];
		float							mRw_y[32];
		float							mRw_z[32];
		float4							mRelativeQuat[32];

		PxU32							mNbStaticContacts[32];
		PxU32							mStaticContactStartIndex[32];

		PxU32							mNbStaticJoints[32];
		PxU32							mStaticJointStartIndex[32];

		PxgSpatialTransformBlock		mPreTransform;
		PxgSpatialTransformBlock		mAccumulatedPose;
		PxgSpatialTransformBlock		mChildPose;
		PxgSpatialTransformBlock		mParentPose;
		float4							mDeltaQ[32];
		float4							mLinDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW[32];
		float4							mInvInertiaXYZ_invMassW[32];
		PxReal							mCfm[32];
		PxU32							mChildrenOffset[32];
		PxU32							mNumChildren[32];

		PxU32							mJointOffset[32];
		PxU32							mParents[32];
		PxU8							mDofs[32];
		PxU8							mJointType[32];
		PxU8							mInvDofIds[6][32];			//mapping from axis to joint offset
		PxU8							mDisableGravity[32];
		PxU8							mRetainsAcceleration[32];

		// PT: padded to 128 for coalesced loads
		PxU32							pad[16];
	};
	PX_COMPILE_TIME_ASSERT((sizeof(PxgArticulationBlockLinkData) & 127)==0);

	struct PxgArticulationBlockSpatialTendonData
	{
	public:
		PxU32	mNumAttachments[32];
		PxU32	mNumConstraints[32];
		PxReal	mStiffness[32];
		PxReal	mDamping[32];
		PxReal	mLimitStiffness[32];
		PxReal  mOffset[32];
	};

	struct PxgArticulationBlockFixedTendonData
	{
	public:
		PxU32	mNumTendonJoints[32];
		PxU32	mNumConstraints[32];
		PxReal	mStiffness[32];
		PxReal	mDamping[32];
		PxReal	mLimitStiffness[32];
		PxReal  mOffset[32];
		PxReal	mLowLimit[32];
		PxReal  mHighLimit[32];
		PxReal	mRestLength[32];
	};

	struct PxgArtiStateDirtyFlag
	{
		enum
		{
			eVEL_DIRTY = 1 << 0,
			eHAS_IMPULSES = 1 << 1
		};
	};

	//This class stores a block of data corresponding to 32 articulations.
	//It is filled in at runtime. 
	struct PxgArticulationBlockData
	{
		PxgSpatialMatrixBlock mInvSpatialArticulatedInertia;
		PxU32 mFlags[32];		// PT: seems to be mainly for PxArticulationFlag::eFIX_BASE so only 1 bit is needed? Merge with mNumLinks?
		PxU32 mNumLinks[32];
		PxU32 mNumSpatialTendons[32];
		PxU32 mNumFixedTendons[32];
		PxU32 mNumMimicJoints[32];
		PxU32 mTotalDofs[32];
		PxU32 mArticulationIndex[32];
		PxReal mSleepThreshold[32];

		//This one is a bit of a hack - it's the common link velocity. 
		PxgSpatialVectorBlock mCommonLinkDeltaVelocity;
		PxU32 mLinkWithDeferredImpulse[32];
		Cm::UnAlignedSpatialVector* mMotionVelocitiesPtr[32];
		PxgSpatialVectorBlock mRootDeferredZ;

		float4	mCOM_TotalInvMassW[32];

		PxU8  mStateDirty[32];				//32 bytes
		PxU32 mTotalSelfConstraintCount;	//4  bytes
		PxU32 mSelfConstraintOffset;		//4 bytes
		// PT: padded to 128 for coalesced loads
		PxU32 pad[22];						//88 bytes padding, bringing this all up to 128 bytes
	};
	PX_COMPILE_TIME_ASSERT((sizeof(PxgArticulationBlockData) & 127)==0);

	struct PxgArticulationInternalConstraintData
	{
		PxgSpatialVectorBlock		mRow0;
		PxgSpatialVectorBlock		mRow1;
		PxgSpatialVectorBlock		mDeltaVA;
		PxgSpatialVectorBlock		mDeltaVB;

		PxReal						mRecipResponse[32];
		PxReal						mResponse[32];

		float2						mLimits_LowLimitX_highLimitY[32];
		float2						mLimitError_LowX_highY[32];
		PxReal						mHighImpulse[32];
		PxReal						mLowImpulse[32];
		PxReal						mMaxJointVelocity[32];
		
		// drive 
		PxReal						mDriveImpulse[32];
		PxReal						mConstraintMaxImpulse[32];
		PxReal						mMaxForce[32];
		PxU32						mDriveType[32];
		PxReal						mMaxEffort[32];
		PxReal						mMaxActuatorVelocity[32];
		PxReal						mVelocityDependentResistance[32];
		PxReal						mSpeedEffortGradient[32];
		PxReal						mExternalEffort[32];

		PxReal						mDriveTargetPos[32];
		PxReal						mArmature[32];

		float4                      mTargetVelPlusInitialBiasX_DriveBiasCoefficientY_VelMultiplierZ_ImpulseMultiplierW[32];

		float						mDriveStiffness[32];
		float						mDamping[32];
		float						mDriveTargetVel[32];
		float						mTargetPosBias[32];

		PxReal						mAccumulatedFrictionImpulse[32];

		//old friction
		PxReal						mMaxDeprecatedFrictionForce[32];
		PxReal						mDeprecatedFrictionCoefficient[32];

		//new friction
		PxReal						mStaticFrictionEffort[32];
		PxReal						mDynamicFrictionEffort[32];
		PxReal						mViscousFrictionCoefficient[32];

		PX_FORCE_INLINE void PX_CUDA_CALLABLE setImplicitDriveDesc
			(const PxU32 threadIndexInWarp,const Dy::ArticulationImplicitDriveDesc& driveDesc)
		{
		    mTargetVelPlusInitialBiasX_DriveBiasCoefficientY_VelMultiplierZ_ImpulseMultiplierW[threadIndexInWarp] =
		        make_float4(driveDesc.driveTargetVelPlusInitialBias, driveDesc.driveBiasCoefficient, driveDesc.driveVelMultiplier, driveDesc.driveImpulseMultiplier);
		    mTargetPosBias[threadIndexInWarp] = driveDesc.driveTargetPosBias;
		}
		PX_FORCE_INLINE PX_CUDA_CALLABLE Dy::ArticulationImplicitDriveDesc getImplicitDriveDesc(const PxU32 threadIndexInWarp) const
		{
			const Dy::ArticulationImplicitDriveDesc driveDesc
			(
		        mTargetVelPlusInitialBiasX_DriveBiasCoefficientY_VelMultiplierZ_ImpulseMultiplierW[threadIndexInWarp].x,
		        mTargetVelPlusInitialBiasX_DriveBiasCoefficientY_VelMultiplierZ_ImpulseMultiplierW[threadIndexInWarp].y,
		        mTargetVelPlusInitialBiasX_DriveBiasCoefficientY_VelMultiplierZ_ImpulseMultiplierW[threadIndexInWarp].z,
		        mTargetVelPlusInitialBiasX_DriveBiasCoefficientY_VelMultiplierZ_ImpulseMultiplierW[threadIndexInWarp].w,
				mTargetPosBias[threadIndexInWarp]
			);
			return driveDesc;
		}
	};
	PX_COMPILE_TIME_ASSERT((sizeof(PxgArticulationInternalConstraintData) & 127)==0);

	struct PxgArticulationInternalTendonConstraintData
	{
		PxgSpatialVectorBlock		mRow0;
		PxgSpatialVectorBlock		mRow1;
		
		PxgSpatialVectorBlock		mDeltaVB;

		PxReal						mRecipResponse[32];	
		PxReal						mAccumulatedLength[32];
		PxReal						mBiasCoefficient[32];
		PxReal						mVelMultiplier[32];
		PxReal						mImpulseMultiplier[32];
		PxReal						mAppliedForce[32];

		PxReal						mLimitBiasCoefficient[32];
		PxReal						mLimitImpulseMultiplier[32];
		PxReal						mLimitAppliedForce[32];

		PxU32						mLink0[32];
		PxU32						mLink1[32];
		PxReal						mDeltaVA[32];

		PxReal						mRestDistance[32];
		PxReal						mLowLimit[32];
		PxReal						mHighLimit[32];

		PX_CUDA_CALLABLE PX_FORCE_INLINE void setTendonImplicitSpringParams(const Dy::TendonImplicitSpringParams& springParams, const PxU32 threadIndexInWarp)
		{		
			mBiasCoefficient[threadIndexInWarp] = springParams.biasCoefficient;
			mVelMultiplier[threadIndexInWarp] = springParams.velMultiplier;
			mImpulseMultiplier[threadIndexInWarp] = springParams.impulseMultiplier;
			mLimitBiasCoefficient[threadIndexInWarp] = springParams.limitBiasCoefficient;
			mLimitImpulseMultiplier[threadIndexInWarp] = springParams.limitImpulseMultiplier;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE Dy::TendonImplicitSpringParams	getTendonImplicitSpringParams(const PxU32 threadIndexInWarp) const
		{
			Dy::TendonImplicitSpringParams springParams(
				mBiasCoefficient[threadIndexInWarp], mVelMultiplier[threadIndexInWarp], mImpulseMultiplier[threadIndexInWarp], 
				mLimitBiasCoefficient[threadIndexInWarp], mLimitImpulseMultiplier[threadIndexInWarp]);
			return springParams;
		}
	};

	struct PxgArticulationBlockDofData
	{
		PxReal						mJointPositions[32];
		PxReal						mJointVelocities[32];
		PxReal						mJointUnconstrainedVelocities[32]; 
		PxReal						mPosJointVelocities[32];
		PxReal						mJointAccel[32];
		PxReal						mQstZ[32];
		PxReal						mQstZIcInternal[32];
		PxReal						mDeferredQstZ[32];
		PxReal						mTmpQstZ[32];

		PxgSpatialVectorBlock		mWorldMotionMatrix;
		PxgSpatialVectorBlock		mLocalMotionMatrix;
		PxgSpatialVectorBlock		mIsInvDW;
		PxgSpatialVectorBlock		mIsW;
		float						mInvStIsT_x[32];
		float						mInvStIsT_y[32];
		float						mInvStIsT_z[32];
		PxU8						mDofIds[32];			//mapping from joint offset to axis
		PxU8						mMotion[32];

		PxgArticulationInternalConstraintData mConstraintData;

		// PT: padded to 128 for coalesced loads
		PxU32							pad[16];
	};
	PX_COMPILE_TIME_ASSERT((sizeof(PxgArticulationBlockDofData) & 127)==0);

	struct PxgArticulationBlockAttachmentData
	{
	public:
		PxVec3	mRelativeOffset[32];
		PxReal	mRestDistance[32];
		PxReal	mCoefficient[32];

		PxU64	mChildrens[32];
		PxU16	mLinkIndex[32];
		PxU32	mParents[32];
		PxReal	mLowLimit[32];
		PxReal	mHighLimit[32];	
	};

	struct PxgArticulationBlockTendonJointData
	{
	public:
		PxU32						mAxis[32];
		PxReal						mCoefficient[32];
		PxReal						mRecipCoefficient[32];

		PxU64						mChildrens[32];
		PxU16						mLinkIndex[32];
		PxU32						mParents[32];
		PxU32						mConstraintId[32];
	};

	struct PxgArticulationBlockInternallMimicJointData
	{
		PxU32 mDofA[32];
		PxU32 mDofB[32];
		//Cache recip effectiveInertia = [J * M^-1 * J^T]  =  [rAA + gearRatio*(rAB + rBA) + gearRatio*gearRatio*rBB]
		//Impulse = [1, gearRatio]^T * [-b + J*v] /[J * M^-1 * J^T + cfm]
		//Impulse = [1, gearRatio]^T * [-b + J*v] / [recipEffectiveInertia + cfm];
		PxReal recipEffectiveInertia[32];
	};

	struct PxgArticulationBlockMimicJointData
	{
		PxU32 mLinkA[32];
		PxU32 mAxisA[32];
		PxU32 mLinkB[32];
		PxU32 mAxisB[32];
		PxReal mGearRatio[32];
		PxReal mOffset[32];
		PxReal mNaturalFrequency[32];
		PxReal mDampingRatio[32];
		PxgArticulationBlockInternallMimicJointData mInternalData;
	};

	struct PxgArticulationData
	{
		// index in the articulation buffer. This is the same as articulationRemapId in the PxgBodySim
		PxU32					index;							
		PxU32					bodySimIndex;					
		PxU32					gpuDirtyFlag;					
		PxU32					updateDirty;					

		PxU16					numLinks;						
		PxU16					numJointDofs;					
		PxU16					numSpatialTendons;				

		PxU16					numFixedTendons;				
		PxU16					numMimicJoints;					
		PxU8					flags;							
		bool					confiDirty;						

		PxU32					numPathToRoots;					
	};
}

#endif
