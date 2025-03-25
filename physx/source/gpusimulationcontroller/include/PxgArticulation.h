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

#ifndef PXG_ARTICULATION_H
#define PXG_ARTICULATION_H

#include "foundation/PxSimpleTypes.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxgArticulationLink.h"
#include "PxgArticulationTendon.h"
#include "PxgCudaBuffer.h"
#include "PxSpatialMatrix.h"
#include "DyFeatherstoneArticulation.h"
#include "foundation/PxUserAllocated.h"
#include "vector_types.h"

namespace physx
{
	class PxGpuSpatialTendonData;
	class PxGpuFixedTendonData;

	namespace Dy
	{
		struct ArticulationJointCore;
		class ArticulationJointCoreData;
		struct ArticulationInternalConstraint;
		struct SpatialSubspaceMatrix;
	}

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
		PxReal						mConstraintMaxForce[32];
		PxReal						mMaxForce[32];
		PxU32						mDriveType[32];

		PxReal						mDriveTargetPos[32];
		PxReal						mArmature[32];

		float4                      mTargetVelPlusInitialBiasX_DriveBiasCoefficientY_VelMultiplierZ_ImpulseMultiplierW[32];

		float						mDriveStiffness[32];
		float						mDamping[32];
		float						mDriveTargetVel[32];
		float						mTargetPosBias[32];
		PxReal						mDriveForce[32];

		PxReal						mAccumulatedFrictionImpulse[32];

		//old friction
		PxReal						mMaxFrictionForce[32];
		PxReal						mFrictionCoefficient[32];

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
		PxgSpatialVectorBlock		mJointAxis;
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

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4324)
#endif
	PX_ALIGN_PREFIX(16)
	class PxgArticulation
	{
	public:
	
		PxgArticulationData						data;							

		PxgArticulationLink*					links;							
		Dy::ArticulationJointCore*				joints;							
		Dy::ArticulationJointCoreData*			jointData;						
		
		Cm::UnAlignedSpatialVector*				motionVelocities;				
		Cm::UnAlignedSpatialVector*				motionAccelerations;			
		Cm::UnAlignedSpatialVector*				linkIncomingJointForces;	
		Cm::UnAlignedSpatialVector*				corioliseVectors;				
		Cm::UnAlignedSpatialVector*				zAForces; // used as temporary propagation buffer in inverseDynamics and to store TGS per substep isolated forces while the solver runs. Not cleared after use.
		
		Cm::UnAlignedSpatialVector*				externalAccelerations;			

		Cm::UnAlignedSpatialVector*				rootPreMotionVelocity;

		PxReal*									jointPositions;					
		PxReal*									jointVelocities;				
		PxReal*									jointAccelerations;				
		PxReal*									jointForce;						
		PxReal*									jointTargetPositions;
		PxReal*									jointTargetVelocities;
		
		PxU32*									jointOffsets;					

		PxSpatialMatrix*						worldSpatialArticulatedInertia;	
		PxSpatialMatrix*						spatialResponseMatrixW;
		PxTransform*							linkBody2Worlds;	
		PxU8*									linkJointRootStateDataBuffer;			
		PxTransform*							linkBody2Actors;
		PxU32*									parents;						
		//Local space motion matrix - constant unless properties are changed
		Dy::SpatialSubspaceMatrix*				motionMatrix;					
		//World space motion matrix - computed from local matrix each frame
		Dy::SpatialSubspaceMatrix*				worldMotionMatrix; // AD: only inverse dynamics now.				

		Cm::UnAlignedSpatialVector*				jointAxis;

		PxReal*									linkWakeCounters;
		PxgArticulationLinkSleepData*			linkSleepData;
		PxgArticulationLinkProp*				linkProps;						
		ArticulationBitField*					children;
		PxU32*									pathToRoot;

		PxQuat*									relativeQuat;

		PxQuat*									tempParentToChilds;
		PxVec3*									tempRs;

		PxGpuSpatialTendonData*					spatialTendonParams;
		PxgArticulationTendon*					spatialTendons;

		PxGpuFixedTendonData*					fixedTendonParams;
		PxgArticulationTendon*					fixedTendons;

		PxReal*									cfms;
		PxReal*									cfmScale;

		Dy::ArticulationMimicJointCore*			mimicJointCores;

		PX_ALIGN(16, PxSpatialMatrix)			invSpatialArticulatedInertiaW;

		Dy::ErrorAccumulator					internalResidualAccumulator; //Internal residual means no errors introduces by contacts or non-articulation joints connected to this instance will be included
		Dy::ErrorAccumulator					contactResidualAccumulator;
	}
	PX_ALIGN_SUFFIX(16);
#if PX_VC
#pragma warning(pop)
#endif

	/*
	\brief We aggregate link, joint, and root link state data into a single char buffer.
	We do this in two ways:
	a) a single buffer for each articulation
	b) a single buffer for all articulations
	The typical pattern of data flow is as follows:
	a) we store state data in a unique device buffer for each articulation.
	b) we copy from the individual device buffers per articulation to the single device buffer for all articulations.
	c) we copy the single buffer for all articulations from device to host 
	d) we distribute state data from the single host buffer on the host to the individual articulation instances on the host.
	The state data that we store is as follows:
	a) link body2Worlds array, link velocities array, link accelerations array, link incoming joint forces array
	b) joint positions array, joint velocities array, joint accelerations array
	d) root link pre-sim velocity
	The struct PxgLinkJointRootStateData contains helper functions for allocating and querying 
	state data buffers.
	*/
	struct PxgArticulationLinkJointRootStateData
	{/**
		\brief Compute the number of bytes required for an articulation with known 
		maximum link count and known maximum dof count.
		\param[in] maxNbLinks is the maximum number of links of any articulation in the ensemble of articulations.
		\param[in] maxNbDofs is the maximum number of dofs of any articulation in the ensemble of articulations.
		\note This does not return an aligned size, use computeStateDataBufferByteSizeAligned16 for that purpose
		\return The number of bytes required to store the state data for an articulation.
		*/
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 computeSingleArticulationStateDataBufferByteSizeRaw
		(const PxU32 maxNbLinks, const PxU32 maxNbDofs)
		{
			PxU32 byteSizePerArt =
				(sizeof(PxTransform) + 3 * sizeof(Cm::UnAlignedSpatialVector)) * maxNbLinks;		//link pose + link velocity + link acceleration + link incoming joint force
			byteSizePerArt += sizeof(PxReal) * maxNbDofs; //joint pos
			byteSizePerArt += sizeof(PxReal) * maxNbDofs; //joint vel
			byteSizePerArt += sizeof(PxReal) * maxNbDofs; //joint accel
			byteSizePerArt += sizeof(Cm::UnAlignedSpatialVector);											//root pre-sim vel
			return byteSizePerArt;
		}

		/**
		\brief Compute the number of bytes required for an ensemble of articulations with known 
		maximum link count and known maximum dof count.
		\param[in] maxNbLinks is the maximum number of links of any articulation in the ensemble of articulations.
		\param[in] maxNbDofs is the maximum number of dofs of any articulation in the ensemble of articulations.
		\param[in] nbArticulations is the number of articulations in the ensemble. 
		\note This may be used to compute the number of bytes required for a single articulation by setting 
		nbArticulations to 1 and setting maxNbLinks etc to be the link and dof count of that articulation.
		\return The number of bytes required to store the state data for an ensemble of articulations.
		*/
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 computeStateDataBufferByteSizeAligned16
		(const PxU32 maxNbLinks, const PxU32 maxNbDofs, const PxU32 nbArticulations)
		{
			const PxU32 byteSizePerArt = computeSingleArticulationStateDataBufferByteSizeRaw(maxNbLinks, maxNbDofs);
			const PxU32 byteSize16PerArt = ((byteSizePerArt + 15) & ~15);						//align the size upwards to the next 16-byte boundary
			return (byteSize16PerArt * nbArticulations);											
		}

		/**
		\brief Return the pointer to a single articulation's state data buffer.
		\param[in] inputBufferForAllArticulations is a pointer to the memory containing the state
		data for the entire ensemble of articulations.
		\param[in] maxNbLinks is the maximum number of links of any articulation in the ensemble of articulations.
		\param[in] maxNbDofs is the maximum number of dofs of any articulation in the ensemble of articulations.
		\param[in] articulationId is the index of a single articulation within the ensemble. 
		\return The pointer to a single articulation's state data buffer.
		*/
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU8* getArticulationStateDataBuffer
		(PxU8* inputBufferForAllArticulations,
		 const PxU32 maxNbLinks, const PxU32 maxNbDofs,
		 const PxU32 articulationId)
		{
			PxU8* singleArticulationStateBuffer =
				inputBufferForAllArticulations + 
				computeStateDataBufferByteSizeAligned16(maxNbLinks, maxNbDofs, articulationId);

			return singleArticulationStateBuffer;
		}

		/**
		\brief Decompose the state data buffer of a single articulation into pointers to arrays of 
		poses, velocities etc.
		\param[in] singleArticulationStateBuffer is the state data buffer for a single articulation.
		\param[in] nbLinks is the number of links for the single articulation. 
		\param[in] nbDofs is the number of dofs for the single articulation. 
		\param[out] linkBody2Worlds is an array of link poses with one element per link.
		\param[out] linkVels is a pointer to an array of link spatial velocities with one element per link.
		\param[out] linkIncomingJointForces is a pointer to an array of link incoming joint forces with one element per link.
		\param[out] jointPositions is a pointer to an array of joint positions with one element per dof.
		\param[out] jointVelocities is a pointer to an array of joint velocities with one element per link.
		\param[out] jointAccelerations is a pointer to an array of joint accelerations with one element per dof.
		\param[out] rootPreVel is a pointer to the pre-sim velocity of the single articulation's root link. 
		*/
		static PX_CUDA_CALLABLE PX_FORCE_INLINE void decomposeArticulationStateDataBuffer
		(PxU8* singleArticulationStateBuffer,
		 const PxU32 nbLinks, const PxU32 nbDofs,
		 PxTransform*& linkBody2Worlds, Cm::UnAlignedSpatialVector*& linkVels, Cm::UnAlignedSpatialVector*& linkAccels, Cm::UnAlignedSpatialVector*& linkIncomingJointForces,
		 PxReal*& jointPositions, PxReal*& jointVelocities, PxReal*& jointAccelerations,
		 Cm::UnAlignedSpatialVector*& rootPreVel)
		{
			PxU8* buffer = singleArticulationStateBuffer;
			linkBody2Worlds = reinterpret_cast<PxTransform*>(buffer);
			buffer += sizeof(PxTransform) * nbLinks;
			linkVels = reinterpret_cast<Cm::UnAlignedSpatialVector*>(buffer);
			buffer += sizeof(Cm::UnAlignedSpatialVector) * nbLinks;
			linkAccels = reinterpret_cast<Cm::UnAlignedSpatialVector*>(buffer);
			buffer += sizeof(Cm::UnAlignedSpatialVector) * nbLinks;
			linkIncomingJointForces = reinterpret_cast<Cm::UnAlignedSpatialVector*>(buffer);
			buffer += sizeof(Cm::UnAlignedSpatialVector) * nbLinks;
			jointPositions = reinterpret_cast<PxReal*>(buffer);
			buffer += sizeof(PxReal) * nbDofs;
			jointVelocities = reinterpret_cast<PxReal*>(buffer);
			buffer += sizeof(PxReal) * nbDofs;
			jointAccelerations = reinterpret_cast<PxReal*>(buffer);
			buffer += sizeof(PxReal) * nbDofs;
			rootPreVel =  reinterpret_cast<Cm::UnAlignedSpatialVector*>(buffer);
			PX_ASSERT(
				singleArticulationStateBuffer + computeStateDataBufferByteSizeAligned16(nbLinks, nbDofs, 1) == 
				reinterpret_cast<PxU8*>(((reinterpret_cast<size_t>(buffer) + sizeof(Cm::UnAlignedSpatialVector) + 15) & ~15)));
		}

		/**
		\brief Compute the pointer to the array of link poses for a single articulation.
		\param[in] singleArticulationStateBuffer is the state data buffer for a single articulation.
		\param[in] nbLinks is the number of links for the single articulation. 
		\param[in] nbDofs is the number of dofs for the single articulation. 
		\return The pointer to the array of link poses for a single articulation. 
		*/
		//Get the body2World array from the state data buffer of a single articulation.
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform* getArticulationBody2Worlds
		(PxU8* singleArticulationStateBuffer, 
		 const PxU32 nbLinks, const PxU32 nbDofs)
		{
			PxTransform* linkBody2Worlds;
			Cm::UnAlignedSpatialVector* linkVels;
			Cm::UnAlignedSpatialVector* linkAccels;
			Cm::UnAlignedSpatialVector* linkIncomingJointForces;
			PxReal* jointPositions;
			PxReal* jointVelocities;
			PxReal* jointAccelerations;
			Cm::UnAlignedSpatialVector* rootPreVel;

			decomposeArticulationStateDataBuffer(
				singleArticulationStateBuffer,
				nbLinks, nbDofs, 
				linkBody2Worlds, linkVels, linkAccels, linkIncomingJointForces,
				jointPositions, jointVelocities, jointAccelerations,
				rootPreVel);
	
			return linkBody2Worlds;
		}
	};

	
	class PxgArticulationBuffer : public PxUserAllocated
	{
	public:

		PxgArticulationBuffer(PxgHeapMemoryAllocatorManager* heapMemoryManager);

		~PxgArticulationBuffer();

		PxgTypedCudaBuffer<PxgArticulationLink>					links;
		PxgTypedCudaBuffer<PxReal>								linkWakeCounters;       //original set to the same as articulation wakeCounter
		PxgTypedCudaBuffer<PxgArticulationLinkSleepData>		linkSleepData;
		PxgTypedCudaBuffer<PxgArticulationLinkProp>				linkProps;
		PxgTypedCudaBuffer<Dy::ArticulationJointCore>			joints;
		PxgTypedCudaBuffer<Dy::ArticulationJointCoreData>		jointData;
		PxgTypedCudaBuffer<Cm::UnAlignedSpatialVector>			corioliseVectors;       //link coriolise vector
		PxgTypedCudaBuffer<Cm::UnAlignedSpatialVector>			zAForces;               //link spatial zero acceleration force/ spatical articulate 
		PxgTypedCudaBuffer<PxU32>								pathToRoots;            //global array store path to root for each link in continuous. Each link should have a start index and numberOfElems

		PxgTypedCudaBuffer<PxGpuSpatialTendonData>				spatialTendonParams;
		PxgTypedCudaBuffer<PxgArticulationTendon>				spatialTendons;
		PxArray<PxgCudaBuffer*>									attachmentFixedData;
		PxArray<PxgCudaBuffer*>									attachmentModData;

		PxgTypedCudaBuffer<PxGpuFixedTendonData>				fixedTendonParams;
		PxgTypedCudaBuffer<PxgArticulationTendon>				fixedTendons;
		PxArray<PxgCudaBuffer*>									tendonJointFixData;
		PxArray<PxgCudaBuffer*>									tendonJointCoefficientData;

		PxgTypedCudaBuffer<Dy::ArticulationMimicJointCore>		mimicJoints;

		PxgTypedCudaBuffer<Cm::UnAlignedSpatialVector>			externalAccelerations;

		PxgTypedCudaBuffer<PxReal>								jointForce;
		PxgTypedCudaBuffer<PxReal>								jointTargetPositions;
		PxgTypedCudaBuffer<PxReal>								jointTargetVelocities;
		PxgTypedCudaBuffer<PxU32>								jointOffsets;
		PxgTypedCudaBuffer<PxU32>								parents;
		PxgTypedCudaBuffer<Dy::SpatialSubspaceMatrix>			motionMatrix;
		PxgTypedCudaBuffer<Dy::SpatialSubspaceMatrix>			motionMatrixW;
		PxgTypedCudaBuffer<Cm::UnAlignedSpatialVector>			jointAxis;

		PxgTypedCudaBuffer<PxSpatialMatrix>						spatialArticulatedInertiaW;
		PxgTypedCudaBuffer<PxSpatialMatrix>						spatialImpulseResponseW;

		//see PxgArticulationLinkJointRootStateData
		PxgCudaBuffer											linkAndJointAndRootStates;

		PxgTypedCudaBuffer<PxTransform>							linkBody2Actors;

		PxgTypedCudaBuffer<ArticulationBitField>				children;
														 
		PxgTypedCudaBuffer<PxQuat>								relativeQuats;
		PxgTypedCudaBuffer<PxReal>								cfms;
		PxgTypedCudaBuffer<PxReal>								cfmScale;

		PxgTypedCudaBuffer<PxQuat>								tempParentToChilds;
		PxgTypedCudaBuffer<PxVec3>								tempRs; 

		PxU32													linkCount;

		PxgHeapMemoryAllocatorManager*							mHeapMemoryManager;
	};

	//Helper function to compute the index of a particular link's deltaV value in the deltaV buffer.
	//We store this in a particular order to try and minimize cache misses
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 computeDeltaVIndex(const PxU32 maxArticulations, const PxU32 maxLinks, 
		const PxU32 articulationID, const PxU32 linkID, const PxU32 slabID)
	{
		return articulationID + linkID * maxArticulations + slabID*maxArticulations*maxLinks;
	}

}

#endif
