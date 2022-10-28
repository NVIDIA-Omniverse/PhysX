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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#include "foundation/PxMathUtils.h"
#include "CmConeLimitHelper.h"
#include "DySolverConstraint1D.h"
#include "DyFeatherstoneArticulation.h"
#include "PxsRigidBody.h"
#include "PxcConstraintBlockStream.h"
#include "DyArticulationContactPrep.h"
#include "DyDynamics.h"
#include "DyArticulationPImpl.h"
#include "DyFeatherstoneArticulationLink.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "PxsIslandSim.h"
#include "common/PxProfileZone.h"
#include <stdio.h>


#ifdef _MSC_VER
#pragma warning(disable:4505)
#endif

namespace physx
{
namespace Dy
{
	void PxcFsFlushVelocity(FeatherstoneArticulation& articulation, Cm::SpatialVectorF* deltaV, bool computeSpatialForces);

	//initialize spatial articualted matrix and coriolis spatial force
	void FeatherstoneArticulation::initLinks(ArticulationData& data,
		const PxVec3& /*gravity*/, ScratchData& scratchData, Cm::SpatialVectorF* Z,
		Cm::SpatialVectorF* DeltaV)
	{
		PX_UNUSED(Z);
		PX_UNUSED(DeltaV);
		//compute individual link's spatial inertia tensor
		//[0, M]
		//[I, 0]
		//computeSpatialInertia(data);

		//compute individual zero acceleration force
		//computeZD(data, gravity, scratchData);
		//compute corolis and centrifugal force
		//computeC(data, scratchData);

		if (data.mLinkCount > 1)
		{
			Cm::SpatialVectorF* za = mArticulationData.getTransmittedForces();
			//copy individual zero acceleration force to mTempData zaForce buffer
			//PxMemCopy(za, mArticulationData.getSpatialZAVectors(), sizeof(Cm::SpatialVectorF) * mArticulationData.getLinkCount());
			for (PxU32 linkID = 0; linkID < mArticulationData.getLinkCount(); ++linkID)
			{
				za[linkID] = mArticulationData.mZAForces[linkID] + mArticulationData.mZAInternalForces[linkID];
			}
		}

		computeArticulatedSpatialInertiaAndZ(data, scratchData);
		//computeArticulatedSpatialInertia(data);
		//computeArticulatedSpatialZ(data, scratchData);

		computeArticulatedResponseMatrix(data);

		/*computeJointSpaceJacobians(data);

		computeJointSpaceDeltaV2(data);*/
	}

#if (FEATHERSTONE_DEBUG && (PX_DEBUG || PX_CHECKED))
	static bool isSpatialVectorEqual(Cm::SpatialVectorF& t0, Cm::SpatialVectorF& t1)
	{
		float eps = 0.0001f;
		bool e0 = PxAbs(t0.top.x - t1.top.x) < eps &&
			PxAbs(t0.top.y - t1.top.y) < eps &&
			PxAbs(t0.top.z - t1.top.z) < eps;

		bool e1 = PxAbs(t0.bottom.x - t1.bottom.x) < eps &&
			PxAbs(t0.bottom.y - t1.bottom.y) < eps &&
			PxAbs(t0.bottom.z - t1.bottom.z) < eps;

		return e0 && e1;
	}

	static bool isSpatialVectorZero(Cm::SpatialVectorF& t0)
	{
		float eps = 0.000001f;

		const bool c0 = PxAbs(t0.top.x) < eps && PxAbs(t0.top.y) < eps && PxAbs(t0.top.z) < eps;
		const bool c1 = PxAbs(t0.bottom.x) < eps && PxAbs(t0.bottom.y) < eps && PxAbs(t0.bottom.z) < eps;

		return c0 && c1;
	}
#endif

	//calculate Is
	void FeatherstoneArticulation::computeIs(ArticulationJointCoreData& jointDatum, ArticulationJointTargetData& /*jointTarget*/,
		const PxU32 linkID)
	{
		Cm::SpatialVectorF* IsW = &mArticulationData.mIsW[jointDatum.jointOffset];
		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			//const Cm::UnAlignedSpatialVector spatialArmature = mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset + ind] * (1.f + jointTarget.armature[ind]);
			//const Cm::UnAlignedSpatialVector tmp = mArticulationData.mWorldSpatialArticulatedInertia[linkID] * spatialArmature;

			const Cm::UnAlignedSpatialVector tmp = mArticulationData.mWorldSpatialArticulatedInertia[linkID] * mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset + ind];
			//const Cm::UnAlignedSpatialVector arm = mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset + ind] * jointTarget.armature[ind];
			IsW[ind].top = tmp.top;// +arm.bottom;
			IsW[ind].bottom = tmp.bottom;// +arm.top;
		}
	}

#if FEATHERSTONE_DEBUG
	static inline PxMat33 Outer(const PxVec3& a, const PxVec3& b)
	{
		return PxMat33(a * b.x, a * b.y, a * b.z);
	}
#endif

	SpatialMatrix FeatherstoneArticulation::computePropagateSpatialInertia_ZA_ZIc(const PxU8 jointType, const ArticulationJointTargetData& jointTarget, const ArticulationJointCoreData& jointDatum,
		const SpatialMatrix& articulatedInertia, const Cm::SpatialVectorF* linkIs, InvStIs& invStIs, Cm::SpatialVectorF* isInvD, const Cm::UnAlignedSpatialVector* motionMatrix,
		const PxReal* jF, const Cm::SpatialVectorF& Z, const Cm::SpatialVectorF& ZIntIc, Cm::SpatialVectorF& ZA, Cm::SpatialVectorF& ZAInt, PxReal* qstZ,
		PxReal* qstZIntIc)
	{
		SpatialMatrix spatialInertia;

		switch (jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		case PxArticulationJointType::eREVOLUTE:
		case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
		{
			const Cm::UnAlignedSpatialVector& sa = motionMatrix[0];

#if FEATHERSTONE_DEBUG
			PxVec3 u = (jointType == PxArticulationJointType::ePRISMATIC) ? sa.bottom : sa.top;

			PxMat33 armatureV = Outer(u, u) * jointTarget.armature[0];

			const PxReal armature =  u.dot(armatureV * u);
#endif

			const Cm::SpatialVectorF& Is = linkIs[0];

			const PxReal stIs = (sa.innerProduct(Is) + jointTarget.armature[0]);

			const PxReal iStIs = ((stIs > 0.f) ? (1.f / stIs) : 0.f);

			invStIs.invStIs[0][0] = iStIs;

			Cm::SpatialVectorF isID = Is * iStIs;

			isInvD[0] = isID;

			//(6x1)Is = [v0, v1]; (1x6)stI = [v1, v0]
			//Cm::SpatialVector stI1(Is1.angular, Is1.linear);
			Cm::SpatialVectorF stI(Is.bottom, Is.top);

			spatialInertia = SpatialMatrix::constructSpatialMatrix(isID, stI);

			const PxReal stZ = sa.innerProduct(Z);
			const PxReal stZInt = sa.innerProduct(ZIntIc);

			//link.qstZIc[ind] = jF[ind] - stZ;
			const PxReal qstZF32 = -stZ;
			qstZ[0] = qstZF32;

			//Joint forces should be momentum preserving, so we add them to the biased system to maintain overall system momentum
			const PxReal qstZF32Int = jF[0] - stZInt;
			qstZIntIc[0] = qstZF32Int;

			ZA += isID * qstZF32;

			ZAInt += isID * qstZF32Int;

			break;
		}
		case PxArticulationJointType::eSPHERICAL:
		{
#if FEATHERSTONE_DEBUG
			//This is for debugging
			Temp6x6Matrix bigInertia(articulatedInertia);
			Temp6x3Matrix bigS(motionMatrix.getColumns());

			Temp6x3Matrix bigIs = bigInertia * bigS;

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::SpatialVectorF tempIs = bigInertia * motionMatrix[ind];

				PX_ASSERT(isSpatialVectorEqual(tempIs, linkIs[ind]));

				PX_ASSERT(bigIs.isColumnEqual(ind, tempIs));

			}
#endif
			PxMat33 D(PxIdentity);
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{

#if FEATHERSTONE_DEBUG
				const Cm::UnAlignedSpatialVector& sa0 = motionMatrix[ind];
				const PxVec3 u = sa0.top;
				PxMat33 armatureV = Outer(u, u) * jointTarget.armature[ind];
				PxVec3 armatureU = armatureV * u;
#endif

				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind2];

#if FEATHERSTONE_DEBUG
					const PxVec3 u1 = sa.top;

					const PxReal armature = u1.dot(armatureU);
#endif
				
					D[ind][ind2] = sa.innerProduct(linkIs[ind]);
				}
				D[ind][ind] += jointTarget.armature[ind];
				
			}

			//PxMat33 invD = SpatialMatrix::invertSym33(D);
			PxMat33 invD = D.getInverse();
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					invStIs.invStIs[ind][ind2] = invD[ind][ind2];
				}
			}

#if FEATHERSTONE_DEBUG
			//debugging
			Temp6x3Matrix bigIsInvD = bigIs * invD;
#endif

			Cm::SpatialVectorF columns[6];
			columns[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[1] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[2] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[3] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[4] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[5] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::SpatialVectorF isID(PxVec3(0.f), PxVec3(0.f));
				const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind];

				const PxReal stZ = sa.innerProduct(Z);
				const PxReal stZInt = sa.innerProduct(ZIntIc);

				//link.qstZIc[ind] = jF[ind] - stZ;
				const PxReal localQstZ = - stZ;
				const PxReal localQstZInt = jF[ind] -stZInt;
				qstZ[ind] = localQstZ;
				qstZIntIc[ind] = localQstZInt;

				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					const Cm::SpatialVectorF& Is = linkIs[ind2];
					isID += Is * invD[ind][ind2];
				}
				columns[0] += isID * linkIs[ind].bottom.x;
				columns[1] += isID * linkIs[ind].bottom.y;
				columns[2] += isID * linkIs[ind].bottom.z;
				columns[3] += isID * linkIs[ind].top.x;
				columns[4] += isID * linkIs[ind].top.y;
				columns[5] += isID * linkIs[ind].top.z;
				isInvD[ind] = isID;

				ZA += isID * localQstZ;
				ZAInt += isID * localQstZInt;

#if FEATHERSTONE_DEBUG
				const bool equal = bigIsInvD.isColumnEqual(ind, isInvD.isInvD[ind]);
				PX_ASSERT(equal);
#endif
			}

#if FEATHERSTONE_DEBUG
			Temp6x6Matrix transpose6x6 = bigInertia.getTranspose();
#endif

			spatialInertia = SpatialMatrix::constructSpatialMatrix(columns);
#if FEATHERSTONE_DEBUG
			Temp6x6Matrix result = bigIsInvD * stI;
			PX_ASSERT(result.isEqual(columns));
#endif
			break;
		}
		default:
			return articulatedInertia;
		}

		//(I - Is*Inv(sIs)*sI)
		spatialInertia = articulatedInertia - spatialInertia;

		return spatialInertia;
	}


	SpatialMatrix FeatherstoneArticulation::computePropagateSpatialInertia_ZA_ZIc_NonSeparated(const PxU8 jointType, const ArticulationJointTargetData& jointTarget, const ArticulationJointCoreData& jointDatum,
		const SpatialMatrix& articulatedInertia, const Cm::SpatialVectorF* linkIs, InvStIs& invStIs, Cm::SpatialVectorF* isInvD, const Cm::UnAlignedSpatialVector* motionMatrix,
		const PxReal* jF, const Cm::SpatialVectorF& Z, Cm::SpatialVectorF& ZA, PxReal* qstZIc)
	{
		SpatialMatrix spatialInertia;

		switch (jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		case PxArticulationJointType::eREVOLUTE:
		case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
		{
			const Cm::UnAlignedSpatialVector& sa = motionMatrix[0];

#if FEATHERSTONE_DEBUG
			PxVec3 u = (jointType == PxArticulationJointType::ePRISMATIC) ? sa.bottom : sa.top;

			PxMat33 armatureV = Outer(u, u) * jointTarget.armature[0];

			const PxReal armature = u.dot(armatureV * u);
#endif

			const Cm::SpatialVectorF& Is = linkIs[0];

			const PxReal stIs = (sa.innerProduct(Is) + jointTarget.armature[0]);

			const PxReal iStIs = (stIs > 1e-10f) ? (1.f / stIs) : 0.f;

			invStIs.invStIs[0][0] = iStIs;

			Cm::SpatialVectorF isID = Is * iStIs;

			isInvD[0] = isID;

			//(6x1)Is = [v0, v1]; (1x6)stI = [v1, v0]
			//Cm::SpatialVector stI1(Is1.angular, Is1.linear);
			Cm::SpatialVectorF stI(Is.bottom, Is.top);

			spatialInertia = SpatialMatrix::constructSpatialMatrix(isID, stI);

			const PxReal stZ = sa.innerProduct(Z);

			//link.qstZIc[ind] = jF[ind] - stZ;
			const PxReal qstZIcF32 = jF[0] - stZ;
			qstZIc[0] = qstZIcF32;

			ZA += isID * qstZIcF32;

			break;
		}
		case PxArticulationJointType::eSPHERICAL:
		{
#if FEATHERSTONE_DEBUG
			//This is for debugging
			Temp6x6Matrix bigInertia(articulatedInertia);
			Temp6x3Matrix bigS(motionMatrix.getColumns());

			Temp6x3Matrix bigIs = bigInertia * bigS;

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::SpatialVectorF tempIs = bigInertia * motionMatrix[ind];

				PX_ASSERT(isSpatialVectorEqual(tempIs, linkIs[ind]));

				PX_ASSERT(bigIs.isColumnEqual(ind, tempIs));

			}
#endif
			PxMat33 D(PxIdentity);
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{

#if FEATHERSTONE_DEBUG
				const Cm::UnAlignedSpatialVector& sa0 = motionMatrix[ind];
				const PxVec3 u = sa0.top;
				PxMat33 armatureV = Outer(u, u) * jointTarget.armature[ind];
				PxVec3 armatureU = armatureV * u;
#endif

				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind2];

#if FEATHERSTONE_DEBUG
					const PxVec3 u1 = sa.top;

					const PxReal armature = u1.dot(armatureU);
#endif

					D[ind][ind2] = sa.innerProduct(linkIs[ind]);
				}
				D[ind][ind] += jointTarget.armature[ind];
				//D[ind][ind] *= 10.f;
			}

			PxMat33 invD = SpatialMatrix::invertSym33(D);
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					invStIs.invStIs[ind][ind2] = invD[ind][ind2];

				}
			}

#if FEATHERSTONE_DEBUG
			//debugging
			Temp6x3Matrix bigIsInvD = bigIs * invD;
#endif

			Cm::SpatialVectorF columns[6];
			columns[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[1] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[2] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[3] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[4] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[5] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::SpatialVectorF isID(PxVec3(0.f), PxVec3(0.f));
				const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind];

				const PxReal stZ = sa.innerProduct(Z);

				//link.qstZIc[ind] = jF[ind] - stZ;
				const PxReal localQstZ = jF[ind] - stZ;
				qstZIc[ind] = localQstZ;

				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					const Cm::SpatialVectorF& Is = linkIs[ind2];
					isID += Is * invD[ind][ind2];
				}
				columns[0] += isID * linkIs[ind].bottom.x;
				columns[1] += isID * linkIs[ind].bottom.y;
				columns[2] += isID * linkIs[ind].bottom.z;
				columns[3] += isID * linkIs[ind].top.x;
				columns[4] += isID * linkIs[ind].top.y;
				columns[5] += isID * linkIs[ind].top.z;
				isInvD[ind] = isID;

				ZA += isID * localQstZ;

#if FEATHERSTONE_DEBUG
				const bool equal = bigIsInvD.isColumnEqual(ind, isInvD.isInvD[ind]);
				PX_ASSERT(equal);
#endif
			}

#if FEATHERSTONE_DEBUG
			Temp6x6Matrix transpose6x6 = bigInertia.getTranspose();
#endif

			spatialInertia = SpatialMatrix::constructSpatialMatrix(columns);
#if FEATHERSTONE_DEBUG
			Temp6x6Matrix result = bigIsInvD * stI;
			PX_ASSERT(result.isEqual(columns));
#endif
			break;
		}
		default:
			spatialInertia.setZero();
			break;
		}

		//(I - Is*Inv(sIs)*sI)
		spatialInertia = articulatedInertia - spatialInertia;

		return spatialInertia;
	}


	SpatialMatrix FeatherstoneArticulation::computePropagateSpatialInertia(const PxU8 jointType, ArticulationJointCoreData& jointDatum,
		const SpatialMatrix& articulatedInertia, const Cm::SpatialVectorF* linkIs, InvStIs& invStIs, Cm::SpatialVectorF* isInvD, const Cm::UnAlignedSpatialVector* motionMatrix)
	{
		SpatialMatrix spatialInertia;

		switch (jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		case PxArticulationJointType::eREVOLUTE:
		case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
		{
			const Cm::UnAlignedSpatialVector& sa = motionMatrix[0];

			const Cm::SpatialVectorF& Is = linkIs[0];

			const PxReal stIs = sa.innerProduct(Is);

			const PxReal iStIs = (stIs > 1e-10f) ? (1.f / stIs) : 0.f;

			invStIs.invStIs[0][0] = iStIs;

			Cm::SpatialVectorF isID = Is * iStIs;

			isInvD[0] = isID;

			//(6x1)Is = [v0, v1]; (1x6)stI = [v1, v0]
			//Cm::SpatialVector stI1(Is1.angular, Is1.linear);
			Cm::SpatialVectorF stI(Is.bottom, Is.top);

			spatialInertia = SpatialMatrix::constructSpatialMatrix(isID, stI);

			break;
		}
		case PxArticulationJointType::eSPHERICAL:
		{
#if FEATHERSTONE_DEBUG
			//This is for debugging
			Temp6x6Matrix bigInertia(articulatedInertia);
			Temp6x3Matrix bigS(motionMatrix.getColumns());

			Temp6x3Matrix bigIs = bigInertia * bigS;

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::SpatialVectorF tempIs = bigInertia * motionMatrix[ind];

				PX_ASSERT(isSpatialVectorEqual(tempIs, linkIs[ind]));

				PX_ASSERT(bigIs.isColumnEqual(ind, tempIs));

			}
#endif
			PxMat33 D(PxIdentity);
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind2];
					D[ind][ind2] = sa.innerProduct(linkIs[ind]);
				}
			}

			PxMat33 invD = SpatialMatrix::invertSym33(D);
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					invStIs.invStIs[ind][ind2] = invD[ind][ind2];

				}
			}

#if FEATHERSTONE_DEBUG
			//debugging
			Temp6x3Matrix bigIsInvD = bigIs * invD;
#endif

			Cm::SpatialVectorF columns[6];
			columns[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[1] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[2] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[3] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[4] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			columns[5] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::SpatialVectorF isID(PxVec3(0.f), PxVec3(0.f));

				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					const Cm::SpatialVectorF& Is = linkIs[ind2];
					isID += Is * invD[ind][ind2];
				}
				columns[0] += isID * linkIs[ind].bottom.x;
				columns[1] += isID * linkIs[ind].bottom.y;
				columns[2] += isID * linkIs[ind].bottom.z;
				columns[3] += isID * linkIs[ind].top.x;
				columns[4] += isID * linkIs[ind].top.y;
				columns[5] += isID * linkIs[ind].top.z;
				isInvD[ind] = isID;

#if FEATHERSTONE_DEBUG
				const bool equal = bigIsInvD.isColumnEqual(ind, isInvD.isInvD[ind]);
				PX_ASSERT(equal);
#endif
			}

#if FEATHERSTONE_DEBUG
			Temp6x6Matrix transpose6x6 = bigInertia.getTranspose();
#endif

			spatialInertia = SpatialMatrix::constructSpatialMatrix(columns);
#if FEATHERSTONE_DEBUG
			Temp6x6Matrix result = bigIsInvD * stI;
			PX_ASSERT(result.isEqual(columns));
#endif
			break;
		}
		default:
			spatialInertia.setZero();
			break;
		}

		//(I - Is*Inv(sIs)*sI)
		spatialInertia = articulatedInertia - spatialInertia;

		return spatialInertia;
	}
	
	void FeatherstoneArticulation::computeArticulatedSpatialInertiaAndZ(ArticulationData& data, ScratchData& scratchData)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationJointCoreData* jointData = data.getJointData();
		ArticulationJointTargetData* jointTargetData = data.getJointTranData();
		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;
		Cm::SpatialVectorF* articulatedZA = scratchData.spatialZAVectors;
		Cm::SpatialVectorF* articulatedZAInt = data.mZAInternalForces.begin();

		PxReal* jointForces = scratchData.jointForces;

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& link = links[linkID];
			
			ArticulationJointCoreData& jointDatum = jointData[linkID];
			ArticulationJointTargetData& jointTarget = jointTargetData[linkID];
			computeIs(jointDatum, jointTarget, linkID);

			//calculate spatial zero acceleration force, this can move out of the loop
			Cm::SpatialVectorF Ic = data.mWorldSpatialArticulatedInertia[linkID] * coriolisVectors[linkID];
			Cm::SpatialVectorF Z = articulatedZA[linkID];// + Ic;
			Cm::SpatialVectorF ZIntIc = articulatedZAInt[linkID] + Ic;

			const PxReal* jF = &jointForces[jointDatum.jointOffset];

			Cm::SpatialVectorF ZA = Z;
			Cm::SpatialVectorF ZAIntIc = ZIntIc;

			//(I - Is*Inv(sIs)*sI)
			//KS - we also bury Articulated ZA force and ZIc force computation in here because that saves
			//us some round-trips to memory!
			SpatialMatrix spatialInertiaW = computePropagateSpatialInertia_ZA_ZIc(link.inboundJoint->jointType, jointTarget,
				jointDatum, data.mWorldSpatialArticulatedInertia[linkID], &data.mIsW[jointDatum.jointOffset], data.mInvStIs[linkID], &data.mIsInvDW[jointDatum.jointOffset],
				&data.mWorldMotionMatrix[jointDatum.jointOffset], jF, Z, ZIntIc, ZA, ZAIntIc, &data.qstZIc[jointDatum.jointOffset], &data.qstZIntIc[jointDatum.jointOffset]);

			//transform spatial inertia into parent space
			FeatherstoneArticulation::translateInertia(constructSkewSymmetricMatrix(data.getRw(linkID)), spatialInertiaW);

			const PxReal minPropagatedInertia = 0.f;

			// Make sure we do not propagate up negative inertias around the principal inertial axes 
			// due to numerical rounding errors
			spatialInertiaW.bottomLeft.column0.x = PxMax(minPropagatedInertia, spatialInertiaW.bottomLeft.column0.x);
			spatialInertiaW.bottomLeft.column1.y = PxMax(minPropagatedInertia, spatialInertiaW.bottomLeft.column1.y);
			spatialInertiaW.bottomLeft.column2.z = PxMax(minPropagatedInertia, spatialInertiaW.bottomLeft.column2.z);

			data.mWorldSpatialArticulatedInertia[link.parent] += spatialInertiaW;

			Cm::SpatialVectorF translatedZA = FeatherstoneArticulation::translateSpatialVector(data.getRw(linkID), ZA);
			Cm::SpatialVectorF translatedZAInt = FeatherstoneArticulation::translateSpatialVector(data.getRw(linkID), ZAIntIc);
			articulatedZA[link.parent] += translatedZA;
			articulatedZAInt[link.parent] += translatedZAInt;
		}

		//cache base link inverse spatial inertia
		data.mWorldSpatialArticulatedInertia[0].invertInertiaV(data.mBaseInvSpatialArticulatedInertiaW);
	}

	void FeatherstoneArticulation::computeArticulatedSpatialInertiaAndZ_NonSeparated(ArticulationData& data, ScratchData& scratchData)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationJointCoreData* jointData = data.getJointData();
		ArticulationJointTargetData* jointTargetData = data.getJointTranData();
		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;
		Cm::SpatialVectorF* articulatedZA = scratchData.spatialZAVectors;

		PxReal* jointForces = scratchData.jointForces;

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& link = links[linkID];

			ArticulationJointCoreData& jointDatum = jointData[linkID];
			ArticulationJointTargetData& jointTarget = jointTargetData[linkID];
			computeIs(jointDatum, jointTarget, linkID);

			//calculate spatial zero acceleration force, this can move out of the loop
			Cm::SpatialVectorF Ic = data.mWorldSpatialArticulatedInertia[linkID] * coriolisVectors[linkID];
			Cm::SpatialVectorF Z = articulatedZA[linkID] + Ic;

			const PxReal* jF = &jointForces[jointDatum.jointOffset];

			Cm::SpatialVectorF ZA = Z;

			//(I - Is*Inv(sIs)*sI)
			//KS - we also bury Articulated ZA force and ZIc force computation in here because that saves
			//us some round-trips to memory!
			SpatialMatrix spatialInertiaW = computePropagateSpatialInertia_ZA_ZIc_NonSeparated(link.inboundJoint->jointType, jointTarget,
				jointDatum, data.mWorldSpatialArticulatedInertia[linkID], &data.mIsW[jointDatum.jointOffset], data.mInvStIs[linkID], &data.mIsInvDW[jointDatum.jointOffset],
				&data.mWorldMotionMatrix[jointDatum.jointOffset], jF, Z, ZA, &data.qstZIc[jointDatum.jointOffset]);

			//transform spatial inertia into parent space
			FeatherstoneArticulation::translateInertia(constructSkewSymmetricMatrix(data.getRw(linkID)), spatialInertiaW);

			data.mWorldSpatialArticulatedInertia[link.parent] += spatialInertiaW;

			Cm::SpatialVectorF translatedZA = FeatherstoneArticulation::translateSpatialVector(data.getRw(linkID), ZA);
			articulatedZA[link.parent] += translatedZA;
		}

		//cache base link inverse spatial inertia
		data.mWorldSpatialArticulatedInertia[0].invertInertiaV(data.mBaseInvSpatialArticulatedInertiaW);
	}


	void FeatherstoneArticulation::computeArticulatedSpatialInertia(ArticulationData& data)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationJointCoreData* jointData = data.getJointData();
		ArticulationJointTargetData* jointTargetData = data.getJointTranData();
		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& link = links[linkID];

			ArticulationJointCoreData& jointDatum = jointData[linkID];
			ArticulationJointTargetData& jointTarget = jointTargetData[linkID];
			computeIs(jointDatum, jointTarget, linkID);

			//(I - Is*Inv(sIs)*sI)
			//KS - we also bury Articulated ZA force and ZIc force computation in here because that saves
			//us some round-trips to memory!
			SpatialMatrix spatialInertiaW = computePropagateSpatialInertia(link.inboundJoint->jointType,
				jointDatum, data.mWorldSpatialArticulatedInertia[linkID], &data.mIsW[jointDatum.jointOffset], data.mInvStIs[linkID], &data.mIsInvDW[jointDatum.jointOffset],
				&data.mWorldMotionMatrix[jointDatum.jointOffset]);

			//transform spatial inertia into parent space
			FeatherstoneArticulation::translateInertia(constructSkewSymmetricMatrix(data.getRw(linkID)), spatialInertiaW);

			data.mWorldSpatialArticulatedInertia[link.parent] += spatialInertiaW;
		}

		//cache base link inverse spatial inertia
		data.mWorldSpatialArticulatedInertia[0].invertInertiaV(data.mBaseInvSpatialArticulatedInertiaW);
	}

	void FeatherstoneArticulation::computeArticulatedResponseMatrix(ArticulationData& data)
	{
		//PX_PROFILE_ZONE("ComputeResponseMatrix", 0);

		//We can work out impulse response vectors by propagating an impulse to the root link, then back down to the child link using existing data.
		//Alternatively, we can compute an impulse response matrix, which is a vector of 6x6 matrices, which can be multiplied by the impulse vector to
		//compute the response. This can be stored in world space, saving transforms. It can also be computed incrementally, meaning it should not be
		//dramatically more expensive than propagating the impulse for a single constraint. Furthermore, this will allow us to rapidly compute the 
		//impulse response with the TGS solver allowing us to improve quality of equality positional constraints by properly reflecting non-linear motion
		//of the articulation rather than approximating it with linear projections.

		//The input expected is a local-space impulse and the output is a local-space impulse response vector

		ArticulationLink* links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		//(1) impulse response vector for root link
		SpatialImpulseResponseMatrix* responsesW = data.getImpulseResponseMatrixWorld();

		if (fixBase)
		{
			//Fixed base, so response is zero
			PxMemZero(responsesW, sizeof(SpatialImpulseResponseMatrix));
		}
		else
		{
			//Compute impulse response matrix. Compute the impulse response of unit responses on all 6 axes...
			const SpatialMatrix& inverseArticulatedInertiaW = data.mBaseInvSpatialArticulatedInertiaW;

			PxMat33 bottomRight = inverseArticulatedInertiaW.getBottomRight();
			

			responsesW[0].rows[0] = Cm::SpatialVectorF(inverseArticulatedInertiaW.topLeft.column0, inverseArticulatedInertiaW.bottomLeft.column0);
			responsesW[0].rows[1] = Cm::SpatialVectorF(inverseArticulatedInertiaW.topLeft.column1, inverseArticulatedInertiaW.bottomLeft.column1);
			responsesW[0].rows[2] = Cm::SpatialVectorF(inverseArticulatedInertiaW.topLeft.column2, inverseArticulatedInertiaW.bottomLeft.column2);
			responsesW[0].rows[3] = Cm::SpatialVectorF(inverseArticulatedInertiaW.topRight.column0, bottomRight.column0);
			responsesW[0].rows[4] = Cm::SpatialVectorF(inverseArticulatedInertiaW.topRight.column1, bottomRight.column1);
			responsesW[0].rows[5] = Cm::SpatialVectorF(inverseArticulatedInertiaW.topRight.column2, bottomRight.column2);

			links[0].cfm *= PxMax(responsesW[0].rows[0].bottom.x, PxMax(responsesW[0].rows[1].bottom.y, responsesW[0].rows[2].bottom.z));
		}

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			PxVec3 offset = data.getRw(linkID);
			const PxU32 jointOffset = data.getJointData(linkID).jointOffset;
			const PxU32 dofCount = data.getJointData(linkID).dof;

			for (PxU32 i = 0; i < 6; ++i)
			{
				//Impulse has to be negated!
				Cm::SpatialVectorF vec = Cm::SpatialVectorF::Zero();
				vec[i] = 1.f;

				Cm::SpatialVectorF temp = -vec;

				ArticulationLink& tLink = links[linkID];
				//(1) Propagate impulse to parent
				PxReal qstZ[3] = { 0.f, 0.f, 0.f };
				Cm::SpatialVectorF Zp = FeatherstoneArticulation::propagateImpulseW(&data.mIsInvDW[jointOffset], offset,
					&data.mWorldMotionMatrix[jointOffset], temp, dofCount, qstZ);				//(2) Get deltaV response for parent
				Cm::SpatialVectorF zR = -responsesW[tLink.parent].getResponse(Zp);

				const Cm::SpatialVectorF deltaV = propagateAccelerationW(offset, data.mInvStIs[linkID],
					&data.mWorldMotionMatrix[jointOffset], zR, dofCount, &data.mIsW[jointOffset], qstZ);

				//Store in local space (required for propagation
				responsesW[linkID].rows[i] = deltaV;
			}

			links[linkID].cfm *= PxMax(responsesW[linkID].rows[0].bottom.x, PxMax(responsesW[linkID].rows[1].bottom.y, responsesW[linkID].rows[2].bottom.z));
		}
	}

	void FeatherstoneArticulation::computeArticulatedSpatialZ(ArticulationData& data, ScratchData& scratchData)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationJointCoreData* jointData = data.getJointData();

		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;
		Cm::SpatialVectorF* articulatedZA = scratchData.spatialZAVectors;

		PxReal* jointForces = scratchData.jointForces;
		
		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& link = links[linkID];
		
			ArticulationJointCoreData& jointDatum = jointData[linkID];

			//calculate spatial zero acceleration force, this can move out of the loop
			Cm::SpatialVectorF Ic = data.mWorldSpatialArticulatedInertia[linkID] * coriolisVectors[linkID];
			Cm::SpatialVectorF ZIc = articulatedZA[linkID] + Ic;

			const PxReal* jF = &jointForces[jointDatum.jointOffset];
			
			Cm::SpatialVectorF ZA = ZIc;
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				const Cm::UnAlignedSpatialVector& sa = data.mWorldMotionMatrix[jointDatum.jointOffset + ind];
				const PxReal stZ = sa.innerProduct(ZIc);

				//link.qstZIc[ind] = jF[ind] - stZ;
				const PxReal qstZic = jF[ind] - stZ;
				data.qstZIc[jointDatum.jointOffset + ind] = qstZic;
				PX_ASSERT(PxIsFinite(qstZic));

				ZA += data.mIsInvDW[jointDatum.jointOffset + ind] * qstZic;
			}
			//accumulate childen's articulated zero acceleration force to parent's articulated zero acceleration
			articulatedZA[link.parent] += FeatherstoneArticulation::translateSpatialVector(data.getRw(linkID), ZA);
		}
	}

	void FeatherstoneArticulation::computeJointSpaceJacobians(ArticulationData& data)
	{
		//PX_PROFILE_ZONE("computeJointSpaceJacobians", 0);
		const PxU32 linkCount = data.getLinkCount();
		const PxU32 dofCount = data.getDofs();

		PxTransform* trans = data.getAccumulatedPoses();

		Cm::SpatialVectorF* jointSpaceJacobians = data.getJointSpaceJacobians();
		Dy::ArticulationLink* links = data.mLinks;
		Dy::ArticulationJointCoreData* jointData = data.mJointData;
		Cm::UnAlignedSpatialVector* worldMotionMatrix = data.mWorldMotionMatrix.begin();

		for (PxU32 linkID = 1; linkID < linkCount; linkID++)
		{
			const PxTransform pose = trans[linkID];
			Cm::SpatialVectorF* myJacobian = &jointSpaceJacobians[linkID*dofCount];

			const PxU32 lastDof = jointData[linkID].jointOffset + jointData[linkID].dof;

			PxMemZero(myJacobian, sizeof(Cm::SpatialVectorF)*lastDof);

			PxU32 link = linkID;

			while (link != 0)
			{
				PxU32 parent = links[link].parent;
				const Dy::ArticulationJointCoreData& jData = jointData[link];
				const PxTransform parentPose = trans[link];

				PxVec3 rw = parentPose.p - pose.p;

				const PxU32 jointOffset = jData.jointOffset;
				const PxU32 dofs = jData.dof;

				Cm::UnAlignedSpatialVector* motionMatrix = &worldMotionMatrix[jointOffset];

				for (PxU32 i = 0; i < dofs; ++i)
				{
					myJacobian[jointOffset + i].top = motionMatrix[i].top;
					myJacobian[jointOffset + i].bottom = motionMatrix[i].bottom + rw.cross(motionMatrix[i].top);
				}

				link = parent;
			}

#if 0
			//Verify the jacobian...

			Cm::SpatialVectorF velocity = FeatherstoneArticulation::translateSpatialVector((trans[0].p - trans[linkID].p), data.mMotionVelocities[0]);
			PxReal* jointVelocity = data.getJointVelocities();

			//KS - a bunch of these dofs can be skipped, we just need to follow path-to-root. However, that may be more expensive than
			//just doing the full multiplication. Let's see how expensive it is now...
			for (PxU32 i = 0; i < lastDof; ++i)
			{
				velocity += myJacobian[i] * jointVelocity[i];
			}

			int bob = 0;
			PX_UNUSED(bob);
#endif			
		}

	}

	void FeatherstoneArticulation::computeJointAccelerationW(ArticulationJointCoreData& jointDatum,
		const Cm::SpatialVectorF& pMotionAcceleration, PxReal* jointAcceleration, const Cm::SpatialVectorF* IsW, const PxU32 linkID,
		const PxReal* qstZIc)
	{
		PxReal tJAccel[6];
		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			//stI * pAcceleration
			const PxReal temp = IsW[ind].innerProduct(pMotionAcceleration);

			tJAccel[ind] = (qstZIc[ind] - temp);
		}

		//calculate jointAcceleration

		const InvStIs& invStIs = mArticulationData.mInvStIs[linkID];

		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			jointAcceleration[ind] = 0.f;
			for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
			{
				jointAcceleration[ind] += invStIs.invStIs[ind2][ind] * tJAccel[ind2];
			}
			//PX_ASSERT(PxAbs(jointAcceleration[ind]) < 5000);
		}
	}

	void FeatherstoneArticulation::computeLinkAcceleration(ArticulationData& data, ScratchData& scratchData, bool doIC)
	{
		const PxU32 linkCount = data.getLinkCount();
		const PxReal dt = data.getDt();
		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		//we have initialized motionVelocity and motionAcceleration to be zero in the root link if
		//fix based flag is raised
		Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;
		Cm::SpatialVectorF* coriolis = scratchData.coriolisVectors;

		PxMemZero(data.mSolverSpatialForces.begin(), data.mSolverSpatialForces.size() * sizeof(Cm::SpatialVectorF));

		if (!fixBase)
		{
			//ArticulationLinkData& baseLinkDatum = data.getLinkData(0);
			SpatialMatrix invInertia = data.mBaseInvSpatialArticulatedInertiaW;//baseLinkDatum.spatialArticulatedInertia.invertInertia();

#if FEATHERSTONE_DEBUG
			SpatialMatrix result = invInertia * baseLinkDatum.spatialArticulatedInertia;

			bool isIdentity = result.isIdentity();

			PX_ASSERT(isIdentity);
			PX_UNUSED(isIdentity);
#endif
			//ArticulationLink& baseLink = data.getLink(0);
			//const PxTransform& body2World = baseLink.bodyCore->body2World;
			Cm::SpatialVectorF accel = -(invInertia * spatialZAForces[0]);
			motionAccelerations[0] = accel;
			Cm::SpatialVectorF deltaV = accel * dt;
			motionVelocities[0] += deltaV;
		}
#if FEATHERSTONE_DEBUG
		else
		{
			PX_ASSERT(isSpatialVectorZero(motionAccelerations[0]));
			PX_ASSERT(isSpatialVectorZero(motionVelocities[0]));
		}
#endif

		/*PxReal* jointAccelerations = data.getJointAccelerations();
		PxReal* jointVelocities = data.getJointVelocities();
		PxReal* jointPositions = data.getJointPositions();*/

		PxReal* jointAccelerations = scratchData.jointAccelerations;
		PxReal* jointVelocities = scratchData.jointVelocities;
		PxReal* jointNewVelocities = mArticulationData.mJointNewVelocity.begin();

		//printf("===========================\n");

		//calculate acceleration
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);

			ArticulationJointCore& joint = *link.inboundJoint;
			PX_UNUSED(joint);

			Cm::SpatialVectorF pMotionAcceleration = FeatherstoneArticulation::translateSpatialVector(-data.getRw(linkID), motionAccelerations[link.parent]);

			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

			//calculate jointAcceleration
			PxReal* jA = &jointAccelerations[jointDatum.jointOffset];
			computeJointAccelerationW(jointDatum, pMotionAcceleration, jA, &data.mIsW[jointDatum.jointOffset], linkID,
				&data.qstZIc[jointDatum.jointOffset]);
			//printf("jA %f\n", jA[0]);

			Cm::SpatialVectorF motionAcceleration = pMotionAcceleration;
			if (doIC)
				motionAcceleration += coriolis[linkID];
			PxReal* jointVelocity = &jointVelocities[jointDatum.jointOffset];
			PxReal* jointNewVelocity = &jointNewVelocities[jointDatum.jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				const PxReal accel = jA[ind];
				PxReal jVel = jointVelocity[ind] + accel * dt;
				jointVelocity[ind] = jVel;
				jointNewVelocity[ind] = jVel;
				motionAcceleration.top += data.mWorldMotionMatrix[jointDatum.jointOffset + ind].top * accel;
				motionAcceleration.bottom += data.mWorldMotionMatrix[jointDatum.jointOffset + ind].bottom * accel;
			}

			//KS - can we just work out velocities by projecting out the joint velocities instead of accumulating all this?
			motionAccelerations[linkID] = motionAcceleration;
			PX_ASSERT(motionAccelerations[linkID].isFinite());
			motionVelocities[linkID] += motionAcceleration * dt;

			/*Cm::SpatialVectorF spatialForce = mArticulationData.mWorldSpatialArticulatedInertia[linkID] * motionAcceleration;

			Cm::SpatialVectorF zaForce = -spatialZAForces[linkID];

			int bob = 0;
			PX_UNUSED(bob);*/
		}
	}


	void FeatherstoneArticulation::computeLinkInternalAcceleration(ArticulationData& data, ScratchData& scratchData)
	{
		
		const PxU32 linkCount = data.getLinkCount();
		const PxReal dt = data.getDt();
		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		//we have initialized motionVelocity and motionAcceleration to be zero in the root link if
		//fix based flag is raised
		Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		Cm::SpatialVectorF* spatialZAInternalForces = data.mZAInternalForces.begin();
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;

		Cm::SpatialVectorF* motionAccelerationInt = data.mMotionAccelerationsInternal.begin();

		Cm::SpatialVectorF momentum0(PxVec3(0.f), PxVec3(0.f));

		PxVec3 COM = data.mCOM;

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			const Cm::SpatialVectorF& vel = motionVelocities[linkID];
			PxReal mass = data.mMasses[linkID];
			momentum0.top += vel.bottom*mass;
		}

		PxVec3 rootVel = momentum0.top * data.mInvSumMass;
		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			PxReal mass = data.mMasses[linkID];
			const Cm::SpatialVectorF& vel = motionVelocities[linkID];
			const PxVec3 offsetMass = (data.getAccumulatedPoses()[linkID].p - COM)*mass;

			const PxVec3 angMom = data.mWorldIsolatedSpatialArticulatedInertia[linkID] * vel.top + offsetMass.cross(motionVelocities[linkID].bottom - rootVel);
			momentum0.bottom += angMom;
		}

		PxReal sumInvMass = data.mInvSumMass;

		if (!fixBase)
		{
			//ArticulationLinkData& baseLinkDatum = data.getLinkData(0);
			SpatialMatrix invInertia = data.mBaseInvSpatialArticulatedInertiaW;//baseLinkDatum.spatialArticulatedInertia.invertInertia();

#if FEATHERSTONE_DEBUG
			SpatialMatrix result = invInertia * baseLinkDatum.spatialArticulatedInertia;

			bool isIdentity = result.isIdentity();

			PX_ASSERT(isIdentity);
			PX_UNUSED(isIdentity);
#endif
			//ArticulationLink& baseLink = data.getLink(0);
			//const PxTransform& body2World = baseLink.bodyCore->body2World;
			Cm::SpatialVectorF accel = -(invInertia * spatialZAInternalForces[0]);
			motionAccelerationInt[0] = accel;
			motionAccelerations[0] += accel;
			Cm::SpatialVectorF deltaV = accel * dt;

			motionVelocities[0] += deltaV;
		}
		else
		{
			motionAccelerationInt[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}

		PxReal* jointAccelerations = scratchData.jointAccelerations;
		PxReal* jointInternalAccelerations = mArticulationData.mJointInternalAcceleration.begin();
		PxReal* jointVelocities = scratchData.jointVelocities;
		PxReal* jointNewVelocities = mArticulationData.mJointNewVelocity.begin();

		//printf("===========================\n");

		//calculate acceleration
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);

			ArticulationJointCore& joint = *link.inboundJoint;
			PX_UNUSED(joint);

			Cm::SpatialVectorF pMotionAcceleration = FeatherstoneArticulation::translateSpatialVector(-data.getRw(linkID), motionAccelerationInt[link.parent]);

			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

			//calculate jointAcceleration
			PxReal* jA = &jointAccelerations[jointDatum.jointOffset];
			PxReal* jIntAccel = &jointInternalAccelerations[jointDatum.jointOffset];
			computeJointAccelerationW(jointDatum, pMotionAcceleration, jIntAccel, &data.mIsW[jointDatum.jointOffset], linkID,
				&data.qstZIntIc[jointDatum.jointOffset]);
			//printf("jA %f\n", jA[0]);

			//KS - TODO - separate integration of coriolis vectors!
			Cm::SpatialVectorF motionAcceleration = pMotionAcceleration + coriolisVectors[linkID];
			PxReal* jointVelocity = &jointVelocities[jointDatum.jointOffset];
			PxReal* jointNewVelocity = &jointNewVelocities[jointDatum.jointOffset];

			Cm::SpatialVectorF mVel = FeatherstoneArticulation::translateSpatialVector(-data.getRw(linkID), motionVelocities[link.parent]);

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				const PxReal accel = jIntAccel[ind];
				PxReal jVel = jointVelocity[ind] + accel * dt;
				jointVelocity[ind] = jVel;
				jointNewVelocity[ind] = jVel;
				motionAcceleration.top += data.mWorldMotionMatrix[jointDatum.jointOffset + ind].top * accel;
				motionAcceleration.bottom += data.mWorldMotionMatrix[jointDatum.jointOffset + ind].bottom * accel;

				mVel.top += data.mWorldMotionMatrix[jointDatum.jointOffset + ind].top * jVel;
				mVel.bottom += data.mWorldMotionMatrix[jointDatum.jointOffset + ind].bottom * jVel;

				jA[ind] += accel;
			}

			//KS - can we just work out velocities by projecting out the joint velocities instead of accumulating all this?
			motionAccelerationInt[linkID] = motionAcceleration;
			motionAccelerations[linkID] += motionAcceleration;
			PX_ASSERT(motionAccelerations[linkID].isFinite());
			Cm::SpatialVectorF velDelta = (motionAcceleration)* dt;
			motionVelocities[linkID] += velDelta;
		}



		if (!fixBase)
		{

			PxVec3 angMomentum1(0.f);
			PxMat33 inertia(PxZero);

			PxVec3 sumLinMomentum(0.f);

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				PxReal mass = data.mMasses[linkID];
				sumLinMomentum += motionVelocities[linkID].bottom * mass;
			}

			rootVel = sumLinMomentum * sumInvMass;

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				PxReal mass = data.mMasses[linkID];
				const PxVec3 offset = data.getAccumulatedPoses()[linkID].p - COM;
				inertia += translateInertia(data.mWorldIsolatedSpatialArticulatedInertia[linkID], mass, offset);

				angMomentum1 += data.mWorldIsolatedSpatialArticulatedInertia[linkID] * motionVelocities[linkID].top 
					+ offset.cross(motionVelocities[linkID].bottom - rootVel) * mass;
			}

			PxMat33 invCompoundInertia = inertia.getInverse();

			PxReal denom0 = angMomentum1.magnitude();
			PxReal num = momentum0.bottom.magnitude();
			PxReal angRatio = denom0 == 0.f ? 1.f : num / denom0;

			PxVec3 deltaAngMom = angMomentum1 * (angRatio - 1.f);

			PxVec3 deltaAng = invCompoundInertia * deltaAngMom;

			if (mSolverDesc.core)
			{
				const PxReal maxAng = mSolverDesc.core->maxAngularVelocity;
				const PxReal maxAngSq = maxAng * maxAng;
				PxVec3 ang = (invCompoundInertia * angMomentum1) + deltaAng;
				if (ang.magnitudeSquared() > maxAngSq)
				{
					PxReal ratio = maxAng / ang.magnitude();
					deltaAng += (ratio - 1.f)*ang;
				}
			}


#if 1
			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				const PxVec3 offset = (data.getAccumulatedPoses()[linkID].p - COM);
				Cm::SpatialVectorF velChange(deltaAng, -offset.cross(deltaAng));
				motionVelocities[linkID] += velChange;
				PxReal mass = data.mMasses[linkID];
				sumLinMomentum += velChange.bottom * mass;
			}
#else
			motionVelocities[0].top += deltaAng;
			motionAccelerations[0] = Cm::SpatialVectorF(deltaAng, PxVec3(0.f));
			//sumLinMomentum = motionVelocities[0].bottom * data.mMasses[0];
			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				Cm::SpatialVectorF velChange = Dy::FeatherstoneArticulation::translateSpatialVector(-data.getRw(linkID), motionAccelerations[data.getLink(linkID).parent]);
				motionVelocities[linkID] += velChange;
				motionAccelerations[linkID] = velChange;

				PxReal mass = data.mMasses[linkID];
				//sumLinMomentum += motionVelocities[linkID].bottom * mass;
				sumLinMomentum += velChange.bottom * mass;
			}
#endif

#if 0
			PxReal denom1 = sumLinMomentum.magnitude();
			PxReal linRatio = PxMin(10.f, denom1 == 0.f ? 1.f : momentum0.top.magnitude() / denom1);

			PxVec3 deltaLinMom = sumLinMomentum * (linRatio - 1.f);


#else
			PxVec3 deltaLinMom = momentum0.top - sumLinMomentum;
#endif
			PxVec3 deltaLin = deltaLinMom * sumInvMass;

			if (mSolverDesc.core)
			{
				const PxReal maxLin = mSolverDesc.core->maxLinearVelocity;
				const PxReal maxLinSq = maxLin * maxLin;
				PxVec3 lin = (sumLinMomentum * sumInvMass) + deltaLin;
				if (lin.magnitudeSquared() > maxLinSq)
				{
					PxReal ratio = maxLin / lin.magnitude();
					deltaLin += (ratio - 1.f)*lin;
				}
			}

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				motionVelocities[linkID].bottom += deltaLin;
			}

#if PX_DEBUG && 0

			const bool validateMomentum = false;
			if (validateMomentum)
			{

				Cm::SpatialVectorF momentum2(PxVec3(0.f), PxVec3(0.f));
				for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
				{
					const PxReal mass = data.mMasses[linkID];
					momentum2.top += motionVelocities[linkID].bottom * mass;
				}
				rootVel = momentum2.top * sumInvMass;
				for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
				{
					const PxReal mass = data.mMasses[linkID];
					const PxVec3 angMom = data.mWorldIsolatedSpatialArticulatedInertia[linkID] * motionVelocities[linkID].top +
						(data.getAccumulatedPoses()[linkID].p - COM).cross(motionVelocities[linkID].bottom - rootVel) * mass;
					momentum2.bottom += angMom;
				}

				static PxU32 count = 0;
				count++;

				printf("LinMom0 = %f, LinMom1 = %f, LinMom2 = %f\n", momentum0.top.magnitude(), sumLinMomentum.magnitude(), momentum2.top.magnitude());
				printf("AngMom0 = %f, AngMom1 = %f, AngMom2 = %f\n", momentum0.bottom.magnitude(), angMomentum1.magnitude(), momentum2.bottom.magnitude());

				printf("%i: Angular Momentum0 (%f, %f, %f), angularMomentum1 (%f, %f, %f), angularMomFinal (%f, %f, %f)\n",
					count, momentum0.bottom.x, momentum0.bottom.y, momentum0.bottom.z, angMomentum1.x, angMomentum1.y, angMomentum1.z,
					momentum2.bottom.x, momentum2.bottom.y, momentum2.bottom.z);
			}

#endif
		}

	}

	void FeatherstoneArticulation::computeJointTransmittedFrictionForce(
		ArticulationData& data, ScratchData& scratchData, Cm::SpatialVectorF* /*Z*/, Cm::SpatialVectorF* /*DeltaV*/)
	{
		//const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = data.getLinkCount() - 1;

		//const PxReal frictionCoefficient =30.5f;
		Cm::SpatialVectorF* transmittedForce = scratchData.spatialZAVectors;

		for (PxU32 linkID = startIndex; linkID > 1; --linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);

			//joint force transmitted from parent to child
			//transmittedForce[link.parent] += data.mChildToParent[linkID] *  transmittedForce[linkID];
			transmittedForce[link.parent] += FeatherstoneArticulation::translateSpatialVector(data.getRw(linkID), transmittedForce[linkID]);
		}

		transmittedForce[0] = Cm::SpatialVectorF::Zero();

		//const PxReal dt = data.getDt();
		//for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		//{
		//	//ArticulationLink& link = data.getLink(linkID);
		//	//ArticulationLinkData& linkDatum = data.getLinkData(linkID);
		//	transmittedForce[linkID] = transmittedForce[linkID] * (frictionCoefficient) * dt;
		//	//transmittedForce[link.parent] -= linkDatum.childToParent * transmittedForce[linkID];
		//}

		//

		//applyImpulses(transmittedForce, Z, DeltaV);

		//PxReal* deltaV = data.getJointDeltaVelocities();
		//PxReal* jointV = data.getJointVelocities();

		//for (PxU32 linkID = 1; linkID < data.getLinkCount(); ++linkID)
		//{
		//	ArticulationJointCoreData& tJointDatum = data.getJointData()[linkID];
		//	for (PxU32 i = 0; i < tJointDatum.dof; ++i)
		//	{
		//		jointV[i + tJointDatum.jointOffset] += deltaV[i + tJointDatum.jointOffset];
		//		deltaV[i + tJointDatum.jointOffset] = 0.f;
		//	}
		//}
	}

	//void FeatherstoneArticulation::computeJointFriction(ArticulationData& data,
	//	ScratchData& scratchData)
	//{
	//	PX_UNUSED(scratchData);
	//	const PxU32 linkCount = data.getLinkCount();
	//	PxReal* jointForces = scratchData.jointForces;
	//	PxReal* jointFrictionForces = data.getJointFrictionForces();
	//	PxReal* jointVelocities = data.getJointVelocities();

	//	const PxReal coefficient = 0.5f;

	//	for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
	//	{
	//		ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
	//		//compute generalized force
	//		PxReal* jFs = &jointForces[jointDatum.jointOffset];
	//		PxReal* jVs = &jointVelocities[jointDatum.jointOffset];
	//		PxReal* jFFs = &jointFrictionForces[jointDatum.jointOffset];

	//		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
	//		{
	//			PxReal sign = jVs[ind] > 0 ? -1.f : 1.f;
	//			jFFs[ind] = coefficient * PxAbs(jFs[ind]) *sign;

	//			//jFFs[ind] = coefficient * jVs[ind];
	//		}
	//	}
	//}

	PxU32 FeatherstoneArticulation::computeUnconstrainedVelocities(
		const ArticulationSolverDesc& desc,
		PxReal dt,
		PxU32& acCount,
		const PxVec3& gravity, 
		Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV,
		const PxReal invLengthScale)
	{

		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
		ArticulationData& data = articulation->mArticulationData;
		data.setDt(dt);

		articulation->computeUnconstrainedVelocitiesInternal(gravity, Z, deltaV, invLengthScale);

		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		return articulation->setupSolverConstraints(data.getLinks(), data.getLinkCount(), fixBase, data, Z, acCount);
	}

	void FeatherstoneArticulation::computeUnconstrainedVelocitiesTGS(
		const ArticulationSolverDesc& desc,
		PxReal dt, const PxVec3& gravity,
		PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV,
		const PxReal invLengthScale)
	{
		PX_UNUSED(contextID);

		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
		ArticulationData& data = articulation->mArticulationData;
		data.setDt(dt);

		articulation->computeUnconstrainedVelocitiesInternal(gravity, Z, DeltaV, invLengthScale);
	}

	//void FeatherstoneArticulation::computeCounteractJointForce(const ArticulationSolverDesc& desc, ScratchData& /*scratchData*/, const PxVec3& gravity)
	//{
	//	const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;

	//	const PxU32 linkCount = mArticulationData.getLinkCount();
	//	const PxU32 totalDofs = mArticulationData.getDofs();
	//	//common data
	//	computeRelativeTransform(mArticulationData);

	//	jcalc(mArticulationData);

	//	computeSpatialInertia(mArticulationData);

	//	DyScratchAllocator allocator(desc.scratchMemory, desc.scratchMemorySize);

	//	ScratchData tempScratchData;
	//	allocateScratchSpatialData(allocator, linkCount, tempScratchData);

	//	//PxReal* gravityJointForce = allocator.alloc<PxReal>(totalDofs);
	//	//{
	//	//	PxMemZero(gravityJointForce, sizeof(PxReal) * totalDofs);

	//	//	//compute joint force due to gravity
	//	//	tempScratchData.jointVelocities = NULL;
	//	//	tempScratchData.jointAccelerations = NULL;
	//	//	tempScratchData.jointForces = gravityJointForce;
	//	//	tempScratchData.externalAccels = NULL;

	//	//	if (fixBase)
	//	//		inverseDynamic(mArticulationData, gravity,tempScratchData);
	//	//	else
	//	//		inverseDynamicFloatingLink(mArticulationData, gravity, tempScratchData);
	//	//}

	//	////PxReal* jointForce = mArticulationData.getJointForces();
	//	//PxReal* tempJointForce = mArticulationData.getTempJointForces();
	//	//{
	//	//	PxMemZero(tempJointForce, sizeof(PxReal) * totalDofs);

	//	//	//compute joint force due to coriolis force
	//	//	tempScratchData.jointVelocities = mArticulationData.getJointVelocities();
	//	//	tempScratchData.jointAccelerations = NULL;
	//	//	tempScratchData.jointForces = tempJointForce;
	//	//	tempScratchData.externalAccels = NULL;

	//	//	if (fixBase)
	//	//		inverseDynamic(mArticulationData, PxVec3(0.f), tempScratchData);
	//	//	else
	//	//		inverseDynamicFloatingLink(mArticulationData, PxVec3(0.f), tempScratchData);
	//	//}

	//	//PxReal* jointForce = mArticulationData.getJointForces();
	//	//for (PxU32 i = 0; i < mArticulationData.getDofs(); ++i)
	//	//{
	//	//	jointForce[i] = tempJointForce[i] - gravityJointForce[i];
	//	//}

	//	//PxReal* jointForce = mArticulationData.getJointForces();
	//	PxReal* tempJointForce = mArticulationData.getTempJointForces();
	//	{
	//		PxMemZero(tempJointForce, sizeof(PxReal) * totalDofs);

	//		//compute joint force due to coriolis force
	//		tempScratchData.jointVelocities = mArticulationData.getJointVelocities();
	//		tempScratchData.jointAccelerations = NULL;
	//		tempScratchData.jointForces = tempJointForce;
	//		tempScratchData.externalAccels = mArticulationData.getExternalAccelerations();

	//		if (fixBase)
	//			inverseDynamic(mArticulationData, gravity, tempScratchData);
	//		else
	//			inverseDynamicFloatingLink(mArticulationData, gravity, tempScratchData);
	//	}

	//	PxReal* jointForce = mArticulationData.getJointForces();
	//	for (PxU32 i = 0; i < mArticulationData.getDofs(); ++i)
	//	{
	//		jointForce[i] = tempJointForce[i];
	//	}
	//}

	void FeatherstoneArticulation::updateArticulation(ScratchData& scratchData,
		const PxVec3& gravity, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV, const PxReal invLengthScale)
	{
		computeRelativeTransformC2P(mArticulationData);

		//computeLinkVelocities(mArticulationData, scratchData);


		//articulation constants
		const PxReal dt = mArticulationData.mDt;
		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		//articulation links
		const PxU32 nbLinks = mArticulationData.mLinkCount;
		const PxTransform* accumulatedPoses = mArticulationData.mAccumulatedPoses.begin();
		const Cm::SpatialVector* externalAccels = mArticulationData.mExternalAcceleration;
		const PxVec3* rws = mArticulationData.mRw.begin();
		Cm::UnAlignedSpatialVector* worldMotionMatrices = mArticulationData.mWorldMotionMatrix.begin();
		ArticulationLinkData* linkData = mArticulationData.mLinksData;
		ArticulationLink* links = mArticulationData.mLinks;
		ArticulationJointCoreData* jointCoreData = mArticulationData.mJointData;
		Cm::SpatialVectorF* motionVelocities = mArticulationData.mMotionVelocities.begin();
		Cm::SpatialVectorF* motionAccelerations = mArticulationData.mMotionAccelerations.begin();
		Cm::SpatialVectorF* coriolisVectors = mArticulationData.mCorioliseVectors.begin();
		Cm::SpatialVectorF* spatialZAForces = mArticulationData.mZAForces.begin();
		Cm::SpatialVectorF* spatialZAInternal = mArticulationData.mZAInternalForces.begin();
		Dy::SpatialMatrix* worldSpatialArticulatedInertias = mArticulationData.mWorldSpatialArticulatedInertia.begin();
		PxMat33* worldIsolatedSpatialArticulatedInertias = mArticulationData.mWorldIsolatedSpatialArticulatedInertia.begin();
		PxReal* linkMasses = mArticulationData.mMasses.begin();
		//articulation joint dofs
		const PxU32 nbJointDofs = mArticulationData.mJointVelocity.size();
		PxReal* jointVelocities = mArticulationData.mJointVelocity.begin();
		//articulation state
		Cm::SpatialVectorF& rootPreMotionVelocity = mArticulationData.mRootPreMotionVelocity;
		PxVec3& com = mArticulationData.mCOM;
		PxReal& invMass = mArticulationData.mInvSumMass;
		computeLinkStates(
			dt, invLengthScale, gravity, fixBase,
			nbLinks,
			accumulatedPoses, externalAccels, rws, worldMotionMatrices, jointCoreData,
			linkData, links, motionAccelerations, 
			motionVelocities, spatialZAForces, spatialZAInternal, coriolisVectors,
			worldIsolatedSpatialArticulatedInertias, linkMasses, worldSpatialArticulatedInertias,
			nbJointDofs,
			jointVelocities,
			rootPreMotionVelocity, com, invMass);
			
		initLinks(mArticulationData, gravity, scratchData, Z, DeltaV);
		computeLinkAcceleration(mArticulationData, scratchData, false);
		computeLinkInternalAcceleration(mArticulationData, scratchData);
	}

	void FeatherstoneArticulation::computeUnconstrainedVelocitiesInternal(
		const PxVec3& gravity,
		Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV, const PxReal invLengthScale)
	{
		//PX_PROFILE_ZONE("Articulations:computeUnconstrainedVelocities", 0);

		//mStaticConstraints.forceSize_Unsafe(0);
		mStatic1DConstraints.forceSize_Unsafe(0);
		mStaticContactConstraints.forceSize_Unsafe(0);

		PxMemZero(mArticulationData.mNbStatic1DConstraints.begin(), mArticulationData.mNbStatic1DConstraints.size()*sizeof(PxU32));
		PxMemZero(mArticulationData.mNbStaticContactConstraints.begin(), mArticulationData.mNbStaticContactConstraints.size() * sizeof(PxU32));
		//const PxU32 linkCount = mArticulationData.getLinkCount();

		mArticulationData.init();

		ScratchData scratchData;
		scratchData.motionVelocities = mArticulationData.getMotionVelocities();
		scratchData.motionAccelerations = mArticulationData.getMotionAccelerations();
		scratchData.coriolisVectors = mArticulationData.getCorioliseVectors();
		scratchData.spatialZAVectors = mArticulationData.getSpatialZAVectors();
		scratchData.jointAccelerations = mArticulationData.getJointAccelerations();
		scratchData.jointVelocities = mArticulationData.getJointVelocities();
		scratchData.jointPositions = mArticulationData.getJointPositions();
		scratchData.jointForces = mArticulationData.getJointForces();
		scratchData.externalAccels = mArticulationData.getExternalAccelerations();

		updateArticulation(scratchData, gravity, Z, DeltaV, invLengthScale);

		
		if (mArticulationData.mLinkCount > 1)
		{
			//use individual zero acceleration force(we copy the initial Z value to the transmitted force buffers in initLink())
			scratchData.spatialZAVectors = mArticulationData.getTransmittedForces();
			computeZAForceInv(mArticulationData, scratchData);
			computeJointTransmittedFrictionForce(mArticulationData, scratchData, Z, DeltaV);
		}

		//the dirty flag is used in inverse dynamic
		mArticulationData.setDataDirty(true);

		//zero zero acceleration vector in the articulation data so that we can use this buffer to accumulated
		//impulse for the contacts/constraints in the PGS/TGS solvers
		//PxMemZero(mArticulationData.getSpatialZAVectors(), sizeof(Cm::SpatialVectorF) * linkCount);

		//Reset deferredQstZ and root deferredZ!
		PxMemZero(mArticulationData.mDeferredQstZ.begin(), sizeof(PxReal)*mArticulationData.getDofs());
		PxMemZero(mArticulationData.mJointConstraintForces.begin(), sizeof(PxReal)*mArticulationData.getDofs());
		mArticulationData.mRootDeferredZ = Cm::SpatialVectorF::Zero();

		// solver progress counters
		maxSolverNormalProgress = 0;
		maxSolverFrictionProgress = 0;
		solverProgress = 0;
		numTotalConstraints = 0;

		for (PxU32 a = 0; a < mArticulationData.getLinkCount(); ++a)
		{
			mArticulationData.mAccumulatedPoses[a] = mArticulationData.getLink(a).bodyCore->body2World;
			mArticulationData.mPreTransform[a] = mArticulationData.getLink(a).bodyCore->body2World;
			mArticulationData.mDeltaQ[a] = PxQuat(PxIdentity);
		}
	}

	void FeatherstoneArticulation::enforcePrismaticLimits(PxReal& jPosition, ArticulationJointCore* joint)
	{
		const PxU32 dofId = joint->dofIds[0];
		if (joint->motion[dofId] == PxArticulationMotion::eLIMITED)
		{
			if (jPosition < (joint->limits[dofId].low))
				jPosition = joint->limits[dofId].low;

			if (jPosition > (joint->limits[dofId].high))
				jPosition = joint->limits[dofId].high;
		}
	}

	PxQuat computeSphericalJointPositions(const PxQuat& relativeQuat,
		const PxQuat& newRot, const PxQuat& pBody2WorldRot,
		PxReal* jPositions, const Cm::UnAlignedSpatialVector* motionMatrix,
		const PxU32 dofs)
	{
		PxQuat newParentToChild = (newRot.getConjugate() * pBody2WorldRot).getNormalized();
		if(newParentToChild.w < 0.f)
			newParentToChild = -newParentToChild;
		//PxQuat newParentToChild = (newRot * pBody2WorldRot.getConjugate()).getNormalized();

		PxQuat jointRotation = newParentToChild * relativeQuat.getConjugate();

		PxReal radians;
		PxVec3 axis;
		jointRotation.toRadiansAndUnitAxis(radians, axis);

		axis *= radians;

		for (PxU32 d = 0; d < dofs; ++d)
		{
			jPositions[d] = -motionMatrix[d].top.dot(axis);
		}

		return newParentToChild;
	}

	PxQuat computeSphericalJointPositions(const PxQuat& /*relativeQuat*/,
		const PxQuat& newRot, const PxQuat& pBody2WorldRot)
	{
		PxQuat newParentToChild = (newRot.getConjugate() * pBody2WorldRot).getNormalized();
		if (newParentToChild.w < 0.f)
			newParentToChild = -newParentToChild;

		/*PxQuat jointRotation = newParentToChild * relativeQuat.getConjugate();

		PxReal radians;
		jointRotation.toRadiansAndUnitAxis(radians, axis);

		axis *= radians;*/

		return newParentToChild;
	}

	void FeatherstoneArticulation::computeAndEnforceJointPositions(ArticulationData& data)
	{
		ArticulationLink* links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		ArticulationJointCoreData* jointData = data.getJointData();

		PxReal* jointPositions = data.getJointPositions();
		
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			ArticulationJointCore* joint = link.inboundJoint;
			ArticulationJointCoreData& jointDatum = jointData[linkID];
			PxReal* jPositions = &jointPositions[jointDatum.jointOffset];
			
			if (joint->jointType == PxArticulationJointType::eSPHERICAL)
			{
				ArticulationLink& pLink = links[link.parent];
				//const PxTransform pBody2World = pLink.bodyCore->body2World;

				const PxU32 dof = jointDatum.dof;

				computeSphericalJointPositions(data.mRelativeQuat[linkID], link.bodyCore->body2World.q,
					pLink.bodyCore->body2World.q, jPositions, &data.getMotionMatrix(jointDatum.jointOffset), dof);
			}
			else if (joint->jointType == PxArticulationJointType::eREVOLUTE)
			{
				PxReal jPos = jPositions[0];

				if (jPos > PxTwoPi)
					jPos -= PxTwoPi*2.f;
				else if (jPos < -PxTwoPi)
					jPos += PxTwoPi*2.f;

				jPos = PxClamp(jPos, -PxTwoPi*2.f, PxTwoPi*2.f);

				jPositions[0] = jPos;

			}
			else if(joint->jointType == PxArticulationJointType::ePRISMATIC)
			{
				enforcePrismaticLimits(jPositions[0], joint);
			}
		}
	}

	void FeatherstoneArticulation::updateJointProperties(PxReal* jointNewVelocities, PxReal* jointVelocities, PxReal* jointAccelerations)
	{
		using namespace Dy;

		const PxU32 dofs = mArticulationData.getDofs();
		const PxReal invDt = 1.f / mArticulationData.getDt();

		for (PxU32 i = 0; i < dofs; ++i)
		{
			const PxReal jNewVel = jointNewVelocities[i];
			PxReal delta = jNewVel - jointVelocities[i];
			jointVelocities[i] = jNewVel;
			jointAccelerations[i] += delta * invDt;
		}
	}

	void FeatherstoneArticulation::recomputeAccelerations(const PxReal dt)
	{
		ArticulationJointCoreData* jointData = mArticulationData.getJointData();
		
		ArticulationLink* links = mArticulationData.getLinks();
		const PxU32 linkCount = mArticulationData.getLinkCount();

		Cm::SpatialVectorF* motionAccels = mArticulationData.getMotionAccelerations();

		PxReal* jAccelerations = mArticulationData.getJointAccelerations();
		
		const PxReal invDt = 1.f / dt;

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		//compute root link accelerations
		if (fixBase)
		{
			motionAccels[0] = Cm::SpatialVectorF::Zero();
		}
		else
		{
			Cm::SpatialVectorF tAccel = (mArticulationData.getMotionVelocity(0) - mArticulationData.mRootPreMotionVelocity) * invDt;
			motionAccels[0].top = tAccel.top;
			motionAccels[0].bottom = tAccel.bottom;

		}

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationJointCoreData& jointDatum = jointData[linkID];

			PxReal* jAccel = &jAccelerations[jointDatum.jointOffset];

			ArticulationLink& link = links[linkID];

			const PxTransform& body2World = link.bodyCore->body2World;

			for (PxU32 i = 0; i < jointDatum.dof; ++i)
			{
				const PxReal accel = jAccel[i];
				const PxVec3 localAngAccel = mArticulationData.mMotionMatrix[jointDatum.jointOffset + i].top * accel;
				const PxVec3 localLinAccel = mArticulationData.mMotionMatrix[jointDatum.jointOffset + i].bottom * accel;

				motionAccels[linkID].top = body2World.rotate(localAngAccel);
				motionAccels[linkID].bottom = body2World.rotate(localLinAccel);
			}
		}
	}

	Cm::SpatialVector FeatherstoneArticulation::recomputeAcceleration(const PxU32 linkId, const PxReal dt) const
	{
		ArticulationLink* links = mArticulationData.getLinks();
		Cm::SpatialVectorF tMotionAccel;
		const PxReal invDt = 1.f / dt;
		if (linkId == 0)
		{
			const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
			if (fixBase)
			{
				tMotionAccel = Cm::SpatialVectorF::Zero();
			}
			else
			{
				Cm::SpatialVectorF tAccel = (mArticulationData.getMotionVelocity(0) - mArticulationData.mRootPreMotionVelocity) * invDt;
				tMotionAccel.top = tAccel.top;
				tMotionAccel.bottom = tAccel.bottom;
			}
		}
		else
		{

			ArticulationJointCoreData* jointData = mArticulationData.getJointData();

			const PxReal* jAccelerations = mArticulationData.getJointAccelerations();

			ArticulationLink& link = links[linkId];

			const PxTransform& body2World = link.bodyCore->body2World;

			const ArticulationJointCoreData& jointDatum = jointData[linkId];

			const PxReal* jAccel = &jAccelerations[jointDatum.jointOffset];

			for (PxU32 i = 0; i < jointDatum.dof; ++i)
			{
				const PxReal accel = jAccel[i];

				tMotionAccel.top = mArticulationData.mMotionMatrix[jointDatum.jointOffset + i].top * accel;
				tMotionAccel.bottom = mArticulationData.mMotionMatrix[jointDatum.jointOffset + i].bottom * accel;

				tMotionAccel.top = body2World.rotate(tMotionAccel.top);
				tMotionAccel.bottom = body2World.rotate(tMotionAccel.bottom);
			}
		}

		return Cm::SpatialVector(tMotionAccel.bottom, tMotionAccel.top);
		
	}


	void FeatherstoneArticulation::propagateLinksDown(ArticulationData& data, PxReal* jointVelocities, PxReal* jointPositions,
		Cm::SpatialVectorF* motionVelocities)
	{
		ArticulationLink* links = mArticulationData.getLinks();

		ArticulationJointCoreData* jointData = mArticulationData.getJointData();

		const PxQuat* const PX_RESTRICT relativeQuats = mArticulationData.mRelativeQuat.begin();

		const PxU32 linkCount = mArticulationData.getLinkCount();

		const PxReal dt = data.getDt();

		
		
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];

			ArticulationJointCoreData& jointDatum = jointData[linkID];
			//ArticulationLinkData& linkDatum = mArticulationData.getLinkData(linkID);
			//const PxTransform oldTransform = preTransforms[linkID];

			ArticulationLink& pLink = links[link.parent];
			const PxTransform pBody2World = pLink.bodyCore->body2World;

			ArticulationJointCore* joint = link.inboundJoint;

			PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
			PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

			PxQuat newParentToChild;
			PxQuat newWorldQ;
			PxVec3 r;

			const PxVec3 childOffset = -joint->childPose.p;
			const PxVec3 parentOffset = joint->parentPose.p;

			PxTransform& body2World = link.bodyCore->body2World;

			const PxQuat relativeQuat = relativeQuats[linkID];

			switch (joint->jointType)
			{
			case PxArticulationJointType::ePRISMATIC:
			{
				const PxReal delta = (jVelocity[0]) * dt;

				PxReal jPos = jPosition[0] + delta;

				enforcePrismaticLimits(jPos, joint);

				jPosition[0] = jPos;

				newParentToChild = relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				const PxVec3& u = data.mMotionMatrix[jointDatum.jointOffset].bottom;

				r = e + d + u * jPos;
				break;
			}
			case PxArticulationJointType::eREVOLUTE:
			{
				//use positional iteration JointVelociy to integrate
				const PxReal delta = (jVelocity[0]) * dt;

				PxReal jPos = jPosition[0] + delta;

				if (jPos > PxTwoPi)
					jPos -= PxTwoPi*2.f;
				else if (jPos < -PxTwoPi)
					jPos += PxTwoPi*2.f;

				jPos = PxClamp(jPos, -PxTwoPi*2.f, PxTwoPi*2.f);
				jPosition[0] = jPos;

				const PxVec3& u = data.mMotionMatrix[jointDatum.jointOffset].top;

				PxQuat jointRotation = PxQuat(-jPos, u);
				if (jointRotation.w < 0)	//shortest angle.
					jointRotation = -jointRotation;

				newParentToChild = (jointRotation * relativeQuat).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;

				PX_ASSERT(r.isFinite());

				break;
			}
			case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
			{
				const PxReal delta = (jVelocity[0]) * dt;

				PxReal jPos = jPosition[0] + delta;

				jPosition[0] = jPos;

				const PxVec3& u = data.mMotionMatrix[jointDatum.jointOffset].top;

				PxQuat jointRotation = PxQuat(-jPos, u);
				if (jointRotation.w < 0)	//shortest angle.
					jointRotation = -jointRotation;

				newParentToChild = (jointRotation * relativeQuat).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;

				PX_ASSERT(r.isFinite());

				break;
			}
			case PxArticulationJointType::eSPHERICAL:  
			{
				if (1)
				{
					const PxTransform oldTransform = data.mAccumulatedPoses[linkID];
#if 0
					Cm::SpatialVectorF worldVel = FeatherstoneArticulation::translateSpatialVector(-mArticulationData.mRw[linkID], motionVelocities[link.parent]);
					for (PxU32 i = 0; i < jointDatum.dof; ++i)
					{
						const PxReal delta = (jVelocity[i]) * dt;
						const PxReal jPos = jPosition[i] + delta;
						jPosition[i] = jPos;
						worldVel.top += data.mWorldMotionMatrix[jointDatum.jointOffset + i].top * jVelocity[i];
						worldVel.bottom += data.mWorldMotionMatrix[jointDatum.jointOffset + i].bottom * jVelocity[i];
					}

					motionVelocities[linkID] = worldVel;

					
#else
					Cm::SpatialVectorF worldVel = motionVelocities[linkID];
					//Cm::SpatialVectorF parentVel = motionVelocities[link.parent];
					//PxVec3 relVel = oldTransform.rotateInv(worldVel.top - parentVel.top);
					//for (PxU32 i = 0; i < jointDatum.dof; ++i)
					//{
					//	const PxReal jVel = mArticulationData.mMotionMatrix[jointDatum.jointOffset + i].top.dot(relVel);
					//	jVelocity[i] = jVel;
					//	/*const PxReal delta = jVel * dt;
					//	jPosition[i] += delta;*/
					//}
#endif

					//Gc and Gc are centre of mass poses of parent(p) and child(c) in the world frame.
					//Introduce Q(v, dt) = PxExp(worldAngVel*dt);
					//Lp and Lc are joint frames of parent(p) and child(c) in the parent and child body frames.

					//The rotational part of Gc will be updated as follows:
					//GcNew.q	= Q(v, dt) * Gc.q
					//We could use GcNew for the new child pose but it isn't in quite the right form
					//to use in a generic way with all the other joint types supported here.
					//Here's what we do.
					//Step 1) add Identity to the rhs.
					//GcNew.q = Gp.q * Gp.q^-1 * Q(v, dt) * Gc.q
					//Step 2) Remember that (A * B^-1) = (B * A ^-1)^-1.
					//Gp.q^-1 * Q(v, dt) * Gc.q = (Q(v, dt) * Gc.q)^-1 * Gp.q
					//GcNew.q = Gp.q * (Q(v, dt) * Gc.q)^-1 * Gp.q
					//Write this out using the variable names used here.
					//The final form is:
					//body2World.q = pBody2World.q * newParent2Child

					//The translational part of GcNew will be updated as follows:
					//GcNew.p	= Gp.p + Gp.q.rotate(Lp.p) - GcNew.q.rotate(Lc.p)
					//			= Gp.p + GcNew.q * (GcNew.q^-1 * Gp.q).rotate(Lp.p) - GcNew.q.rotate(Lc.p)
					//			= Gp.p + GcNew.q.rotate((GcNew.q^-1 * Gp.q).rotate(Lp.p) - GcNew.q.rotate(Lc.p)
					//			= Gp.p + GcNew.q.rotate((GcNew.q^-1 * Gp.q).rotate(Lp.p) - Lc.p)
					//Write this out using the variable names used here.
					//body2World.p = pBody2World.p + body2World.q.rotate(newParent2Child.rotate(parentOffset) + childOffset)
					//Put r = newParent2Child.rotate(parentOffset) + childOffset
					//and we have the final form used here:
					//body2World.p = pBody2World.p + body2World.q.rotate(r)

					//Now let's think about the rotation angles. 
					//Imagine that the joint frames are aligned in the world frame. 
					//The pose(Gc0) of the child body in the world frame will satisfy:
			        	//Gp * Lp = Gc0 * Lc
        				//We can solve for Gc0:
			        	//Gc0 = Gp * Lp * Lc^-1 
			       	 //Gc0 = Gp * (Lc * Lp^-1)^-1
	        			//Now compute the rotation J that rotates from Gc0 to GcNew. 
					//We seek a rotation J in the child body frame (in the aligned state so at Gc0) that satisfies:
					//Gc0 * J = GcNew
					//Let's actually solve for J^-1 (because that's what we do here).
					//J^-1 =  GcNew^-1 *  Gp * (Lc * Lp^-1)^-1    
					//From J^-1 we can retrieve three rotation angles in the child body frame. 
					//We actually want the angles for J. We observe that 
			        	//toAngles(J^-1) = -toAngles(J)
					//Our rotation angles r_b commensurate with J are then:
					//r_b = -toAngles(J^-1)
       				//From r_b we can compute the angles r_j in the child joint frame.
					// r_j = Lc.rotateInv(r_b)
					//Remember that we began our calculation with aligned frames. 
					//We can equally apply r_j to the parent joint frame and achieve the same outcome.
										
					//GcNew = Q(v, dt) * Gc.q
					PxVec3 worldAngVel = worldVel.top;
					newWorldQ = PxExp(worldAngVel*dt) * oldTransform.q;

					//GcNew^-1 * Gp
					newParentToChild = computeSphericalJointPositions(relativeQuat, newWorldQ,
						pBody2World.q);

					//J^-1 = GcNew^-1 * Gp * (Lc * Lp^-1)^-1    
					PxQuat jointRotation = newParentToChild * relativeQuat.getConjugate();
					if(jointRotation.w < 0.0f)			
						jointRotation = -jointRotation;

					//PxVec3 axis = toRotationVector(jointRotation);
					/*PxVec3 axis = jointRotation.getImaginaryPart();
					
					for (PxU32 i = 0; i < jointDatum.dof; ++i)
					{
						PxVec3 sa = data.getMotionMatrix(jointDatum.jointOffset + i).top;
						PxReal angle = -compAng(PxVec3(sa.x, sa.y, sa.z).dot(axis), jointRotation.w);
						jPosition[i] = angle;
					}*/

					//r_j = -Lc.rotateInv(r_b)
					PxVec3 axis; PxReal angle;
					jointRotation.toRadiansAndUnitAxis(angle, axis);
					axis *= angle;
					for (PxU32 i = 0; i < jointDatum.dof; ++i)
					{
						PxVec3 sa = mArticulationData.getMotionMatrix(jointDatum.jointOffset + i).top;
						PxReal ang = -sa.dot(axis);
						jPosition[i] = ang;
					}

					const PxVec3 e = newParentToChild.rotate(parentOffset);
					const PxVec3 d = childOffset;
					r = e + d;
				}
				break;
			}
			case PxArticulationJointType::eFIX:
			{
				//this is fix joint so joint don't have velocity
				newParentToChild = relativeQuat;

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				r = e + d;
				break;
			}
			default:
				break;
			}

			body2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
			body2World.p = pBody2World.p + body2World.q.rotate(r);

			PX_ASSERT(body2World.isSane());
			PX_ASSERT(body2World.isValid());	
		}
	}

	void FeatherstoneArticulation::updateBodies(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV, PxReal dt)
	{
		updateBodies(static_cast<FeatherstoneArticulation*>(desc.articulation), tempDeltaV, dt, true);
	}

	void FeatherstoneArticulation::updateBodiesTGS(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV, PxReal dt)
	{
		updateBodies(static_cast<FeatherstoneArticulation*>(desc.articulation), tempDeltaV, dt, false);
	}

	void FeatherstoneArticulation::updateBodies(FeatherstoneArticulation* articulation, Cm::SpatialVectorF* tempDeltaV, PxReal dt, bool integrateJointPositions)
	{		
		ArticulationData& data = articulation->mArticulationData;
		ArticulationLink* links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		Cm::SpatialVectorF* motionVelocities = data.getMotionVelocities();
		Cm::SpatialVectorF* posMotionVelocities = data.getPosIterMotionVelocities();

		Cm::SpatialVector* externalAccels = data.getExternalAccelerations();
		Cm::SpatialVector zero = Cm::SpatialVector::zero();

		data.setDt(dt);

		const PxU32 nbSensors = data.mNbSensors;

		bool doForces = data.getArticulationFlags() & PxArticulationFlag::eCOMPUTE_JOINT_FORCES
			|| nbSensors;

		//update joint velocities/accelerations due to contacts/constraints.
		if (data.mJointDirty)
		{
			//update delta joint velocity and motion velocity due to velocity iteration changes
			
			//update motionVelocities
			PxcFsFlushVelocity(*articulation, tempDeltaV, doForces);
		}

		Cm::SpatialVectorF momentum0 = Cm::SpatialVectorF::Zero();
		PxVec3 posMomentum(0.f);

		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		//PGS
		if (!fixBase)
		{
			const PxVec3 COM = data.mCOM;

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				PxReal mass = data.mMasses[linkID];
				momentum0.top += motionVelocities[linkID].bottom * mass;
				posMomentum += posMotionVelocities[linkID].bottom * mass;
			}

			PxVec3 rootVel = momentum0.top * data.mInvSumMass;

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				PxReal mass = data.mMasses[linkID];
				PxVec3 offsetMass = (data.mPreTransform[linkID].p - COM)*mass;
				PxVec3 angMom = (data.mWorldIsolatedSpatialArticulatedInertia[linkID] * motionVelocities[linkID].top) +
					offsetMass.cross(motionVelocities[linkID].bottom - rootVel);
				momentum0.bottom += angMom;
			}
		}


		if (!integrateJointPositions)
		{
			//TGS
			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				links[linkID].bodyCore->body2World = data.mAccumulatedPoses[linkID].getNormalized();
			}

			articulation->computeAndEnforceJointPositions(data);
		}
		else
		{

			if (!fixBase)
			{
				const PxTransform& preTrans = data.mAccumulatedPoses[0];

				const Cm::SpatialVectorF& posVel = data.getPosIterMotionVelocity(0);

				updateRootBody(posVel, preTrans, data, dt);
			}
			//using the original joint velocities and delta velocities changed in the positional iter to update joint position/body transform
			articulation->propagateLinksDown(data, data.getPosIterJointVelocities(), data.getJointPositions(), data.getPosIterMotionVelocities());

		}
		//Fix up momentum based on changes in pos. Only currently possible with non-fixed base

		

		if (!fixBase)
		{
			PxVec3 COM = data.mLinks[0].bodyCore->body2World.p * data.mMasses[0];
			data.mAccumulatedPoses[0] = data.mLinks[0].bodyCore->body2World;

			PxVec3 sumLinMom = data.mMotionVelocities[0].bottom * data.mMasses[0];
			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				PxU32 parent = data.mLinks[linkID].parent;

				const PxTransform childPose = data.mLinks[linkID].bodyCore->body2World;

				data.mAccumulatedPoses[linkID] = childPose;

				PxVec3 rw = childPose.p - data.mAccumulatedPoses[parent].p;

				data.mRw[linkID] = rw;

				ArticulationJointCoreData& jointDatum = data.mJointData[linkID];

				const PxReal* jVelocity = &data.mJointNewVelocity[jointDatum.jointOffset];

				Cm::SpatialVectorF vel = FeatherstoneArticulation::translateSpatialVector(-rw, data.mMotionVelocities[parent]);
				Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
				for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
				{
					PxReal jVel = jVelocity[ind];
					//deltaV += data.mWorldMotionMatrix[jointDatum.jointOffset + ind] * jVel;
					deltaV += data.mMotionMatrix[jointDatum.jointOffset + ind] * jVel;
				}

				vel.top += childPose.rotate(deltaV.top);
				vel.bottom += childPose.rotate(deltaV.bottom);

				data.mMotionVelocities[linkID] = vel;

				PxReal mass = data.mMasses[linkID];
				COM += childPose.p * mass;
				sumLinMom += vel.bottom * mass;
			}

			COM *= data.mInvSumMass;

			PxMat33 sumInertia(PxZero);

			PxVec3 sumAngMom(0.f);

			PxVec3 rootLinVel = sumLinMom * data.mInvSumMass;



			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				PxReal mass = data.mMasses[linkID];

				const PxVec3 offset = data.mAccumulatedPoses[linkID].p - COM;
				PxMat33 inertia;
				PxMat33 R(data.mAccumulatedPoses[linkID].q);
				const PxVec3 invInertiaDiag = data.getLink(linkID).bodyCore->inverseInertia;
				PxVec3 inertiaDiag(1.f / invInertiaDiag.x, 1.f / invInertiaDiag.y, 1.f / invInertiaDiag.z);

				const PxVec3 offsetMass = offset * mass;

				Cm::transformInertiaTensor(inertiaDiag, R, inertia);
				//Only needed for debug validation
#if PX_DEBUG
				data.mWorldIsolatedSpatialArticulatedInertia[linkID] = inertia;
#endif
				sumInertia += translateInertia(inertia, mass, offset);
				sumAngMom += inertia * motionVelocities[linkID].top;
				sumAngMom += offsetMass.cross(motionVelocities[linkID].bottom - rootLinVel);
			}

			PxMat33 invSumInertia = sumInertia.getInverse();

			PxReal aDenom = sumAngMom.magnitude();
			PxReal angRatio = aDenom == 0.f ? 0.f : momentum0.bottom.magnitude() / aDenom;
			PxVec3 angMomDelta = sumAngMom * (angRatio - 1.f);

			PxVec3 angDelta = invSumInertia * angMomDelta;

#if 0
			motionVelocities[0].top += angDelta;

			Cm::SpatialVectorF* motionAccelerations = data.getMotionAccelerations();

			motionAccelerations[0] = Cm::SpatialVectorF(angDelta, PxVec3(0.f));

			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				Cm::SpatialVectorF deltaV = Dy::FeatherstoneArticulation::translateSpatialVector(-data.getRw(linkID), motionAccelerations[data.getLink(linkID).parent]);
				motionVelocities[linkID] += deltaV;
				motionAccelerations[linkID] = deltaV;

				sumLinMom += deltaV.bottom*data.mMasses[linkID];
			}
#else

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				const PxVec3 offset = (data.getAccumulatedPoses()[linkID].p - COM);
				Cm::SpatialVectorF velChange(angDelta, -offset.cross(angDelta));
				motionVelocities[linkID] += velChange;
				PxReal mass = data.mMasses[linkID];
				sumLinMom += velChange.bottom * mass;
			}
#endif

			PxVec3 linDelta = (momentum0.top - sumLinMom)*data.mInvSumMass;

			

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				motionVelocities[linkID].bottom += linDelta;
			}

			//if (integrateJointPositions)
			{
				PxVec3 predictedCOM = data.mCOM + posMomentum * (data.mInvSumMass * dt);
				PxVec3 posCorrection = predictedCOM - COM;

				for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
				{
					ArticulationLink& link = links[linkID];
					link.bodyCore->body2World.p += posCorrection;
				}

				COM += posCorrection;
			}

#if PX_DEBUG && 0
			const bool validateMomentum = false;
			if (validateMomentum)
			{

				PxVec3 rootVel = sumLinMom * data.mInvSumMass + linDelta;
				
				Cm::SpatialVectorF momentum2(PxVec3(0.f), PxVec3(0.f));
				for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
				{
					const PxReal mass = data.mMasses[linkID];
					PxVec3 offsetMass = (data.getLink(linkID).bodyCore->body2World.p - COM) * mass;
					const PxVec3 angMom = data.mWorldIsolatedSpatialArticulatedInertia[linkID] * motionVelocities[linkID].top +
						offsetMass.cross(motionVelocities[linkID].bottom - rootVel);
					momentum2.bottom += angMom;
					momentum2.top += motionVelocities[linkID].bottom * mass;
				}


				printf("COM = (%f, %f, %f)\n", COM.x, COM.y, COM.z);
				printf("%i: linMom0 %f, linMom1 %f, angMom0 %f, angMom1 %f\n\n\n",
					count, momentum0.top.magnitude(), momentum2.top.magnitude(),
					momentum0.bottom.magnitude(), momentum2.bottom.magnitude());
			}
#endif
		}


		
		{
			//update joint velocity/accelerations
			PxReal* jointVelocities = data.getJointVelocities();
			PxReal* jointAccelerations = data.getJointAccelerations();
			PxReal* jointNewVelocities = data.getJointNewVelocities();

			articulation->updateJointProperties(jointNewVelocities, jointVelocities, jointAccelerations);
		}

		const PxReal invDt = 1.f/dt;

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			PxsBodyCore* bodyCore = link.bodyCore;

			bodyCore->linearVelocity = motionVelocities[linkID].bottom;
			bodyCore->angularVelocity = motionVelocities[linkID].top;
			//zero external accelerations
			if(!(link.bodyCore->mFlags & PxRigidBodyFlag::eRETAIN_ACCELERATIONS))
				externalAccels[linkID] = zero;
		}

		if (doForces)
		{
			data.mSolverSpatialForces[0] *= invDt;
			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				data.mSolverSpatialForces[linkID] = (data.mWorldSpatialArticulatedInertia[linkID] * data.mSolverSpatialForces[linkID]) * invDt;
			}


			if (data.getArticulationFlags() & PxArticulationFlag::eCOMPUTE_JOINT_FORCES)
			{
				//const PxU32 dofCount = data.getDofs();
				PxReal* constraintForces = data.getJointConstraintForces();
				for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
				{
					const Cm::SpatialVectorF spatialForce = data.mSolverSpatialForces[linkID] - (data.mZAForces[linkID] + data.mZAInternalForces[linkID]);

					ArticulationJointCoreData& jointDatum = data.mJointData[linkID];
					const PxU32 offset = jointDatum.jointOffset;
					const PxU32 dofCount = jointDatum.dof;

					for (PxU32 i = 0; i < dofCount; ++i)
					{
						const PxReal jointForce = data.mMotionMatrix[offset + i].innerProduct(spatialForce);
						constraintForces[offset + i] = jointForce;
					}
				}
			}


			for (PxU32 s = 0; s < nbSensors; ++s)
			{
				ArticulationSensor* sensor = data.mSensors[s];
				const PxU32 linkID = sensor->mLinkID;
				const PxTransform& transform = data.mPreTransform[linkID];
				const PxTransform& relTrans = sensor->mRelativePose;

				//Offset to translate the impulse by
				const PxVec3 offset = transform.rotate(relTrans.p);

				Cm::SpatialVectorF spatialForce(PxVec3(0.f), PxVec3(0.f));

				if (sensor->mFlags & PxArticulationSensorFlag::eCONSTRAINT_SOLVER_FORCES)
					spatialForce += data.mSolverSpatialForces[linkID];
				if (sensor->mFlags & PxArticulationSensorFlag::eFORWARD_DYNAMICS_FORCES)
					spatialForce -= (data.mZAForces[linkID] + data.mZAInternalForces[linkID]);

				// translate from body to sensor frame (offset is body->sensor)
				spatialForce = translateSpatialVector(-offset, spatialForce);

				if (sensor->mFlags & PxArticulationSensorFlag::eWORLD_FRAME)
				{
					data.mSensorForces[s].force = spatialForce.top;
					data.mSensorForces[s].torque = spatialForce.bottom;
				}
				else
				{
					//Now we need to rotate into the sensor's frame. Forces are currently reported in world frame
					const PxQuat rotate = transform.q * relTrans.q;
					data.mSensorForces[s].force = rotate.rotateInv(spatialForce.top);
					data.mSensorForces[s].torque = rotate.rotateInv(spatialForce.bottom);
				}
			}
		}
	}

	void FeatherstoneArticulation::updateRootBody(const Cm::SpatialVectorF& motionVelocity, 
		const PxTransform& preTransform, ArticulationData& data, const PxReal dt)
	{
		ArticulationLink* links = data.getLinks();
		//body2World store new body transform integrated from solver linear/angular velocity

		PX_ASSERT(motionVelocity.top.isFinite());
		PX_ASSERT(motionVelocity.bottom.isFinite());

		ArticulationLink& baseLink = links[0];

		PxsBodyCore* baseBodyCore = baseLink.bodyCore;

		//(1) project the current body's velocity (based on its pre-pose) to the geometric COM that we're integrating around...

		PxVec3 comLinVel = motionVelocity.bottom;

		//using the position iteration motion velocity to compute the body2World
		PxVec3 newP = (preTransform.p) + comLinVel * dt;

		PxQuat deltaQ = PxExp(motionVelocity.top*dt);

		baseBodyCore->body2World = PxTransform(newP, (deltaQ* preTransform.q).getNormalized());

		PX_ASSERT(baseBodyCore->body2World.isFinite() && baseBodyCore->body2World.isValid());
	}


	void FeatherstoneArticulation::getJointAcceleration(const PxVec3& gravity, PxArticulationCache& cache)
	{
		PX_SIMD_GUARD
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Articulation::getJointAcceleration() commonInit need to be called first to initialize data!");
			return;
		}

		const PxU32 linkCount = mArticulationData.getLinkCount();
		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ScratchData scratchData;
		PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

		scratchData.jointVelocities = cache.jointVelocity;
		scratchData.jointForces = cache.jointForce;

		//compute individual link's spatial inertia tensor
		//[0, M]
		//[I, 0]
		computeSpatialInertia(mArticulationData);
	
		computeLinkVelocities(mArticulationData, scratchData);

		//compute individual zero acceleration force
		computeZ(mArticulationData, gravity, scratchData);
		//compute corolis and centrifugal force
		computeC(mArticulationData, scratchData);

		computeArticulatedSpatialInertiaAndZ_NonSeparated(mArticulationData, scratchData);

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		//we have initialized motionVelocity and motionAcceleration to be zero in the root link if
		//fix based flag is raised
		//ArticulationLinkData& baseLinkDatum = mArticulationData.getLinkData(0);

		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;

		if (!fixBase)
		{
			SpatialMatrix inverseArticulatedInertia = mArticulationData.mWorldSpatialArticulatedInertia[0].getInverse();
			motionAccelerations[0] = -(inverseArticulatedInertia * spatialZAForces[0]);
		}
#if FEATHERSTONE_DEBUG
		else
		{
			PX_ASSERT(isSpatialVectorZero(motionAccelerations[0]));
		}
#endif

		PxReal* jointAccelerations = cache.jointAcceleration;
		//calculate acceleration
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = mArticulationData.getLink(linkID);

			//SpatialTransform p2C = linkDatum.childToParent.getTranspose();
			//Cm::SpatialVectorF pMotionAcceleration = mArticulationData.mChildToParent[linkID].transposeTransform(motionAccelerations[link.parent]);

			Cm::SpatialVectorF pMotionAcceleration = FeatherstoneArticulation::translateSpatialVector(-mArticulationData.getRw(linkID), motionAccelerations[link.parent]);

			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
			//calculate jointAcceleration
			PxReal* jA = &jointAccelerations[jointDatum.jointOffset];
			computeJointAccelerationW(jointDatum, pMotionAcceleration, jA, &mArticulationData.mIsW[jointDatum.jointOffset], linkID,
				&mArticulationData.qstZIc[jointDatum.jointOffset]);

			Cm::SpatialVectorF motionAcceleration(PxVec3(0.f), PxVec3(0.f));

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				motionAcceleration.top += mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset + ind].top * jA[ind];
				motionAcceleration.bottom += mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset + ind].bottom * jA[ind];
			}

			motionAccelerations[linkID] = pMotionAcceleration + coriolisVectors[linkID] + motionAcceleration;
			PX_ASSERT(motionAccelerations[linkID].isFinite());
		}

		allocator->free(tempMemory);
	}

}//namespace Dy
}
