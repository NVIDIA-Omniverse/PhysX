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

#include "DyFeatherstoneArticulation.h"
#include "DyDynamics.h"
#include "common/PxProfileZone.h"
#include <stdio.h>

#ifdef _MSC_VER
#pragma warning(disable:4505)
#endif

namespace physx
{
namespace Dy
{
	void PxcFsFlushVelocity(FeatherstoneArticulation& articulation, Cm::SpatialVectorF* deltaV);

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

#if FEATHERSTONE_DEBUG
	static inline PxMat33 Outer(const PxVec3& a, const PxVec3& b)
	{
		return PxMat33(a * b.x, a * b.y, a * b.z);
	}
#endif

	SpatialMatrix FeatherstoneArticulation::computePropagateSpatialInertia_ZA_ZIc
	(const PxArticulationJointType::Enum jointType, const PxU8 nbJointDofs,
	 const Cm::UnAlignedSpatialVector* jointMotionMatricesW, const Cm::SpatialVectorF* jointISW, 	
	 const PxReal* jointTargetArmatures,
	 const PxU8* dofIds, const PxReal* jointExternalForces, 
	 const SpatialMatrix& linkArticulatedInertiaW, 
	 const Cm::SpatialVectorF& linkZExtW, const Cm::SpatialVectorF& linkZIntIcW, 
	 InvStIs& linkInvStISW, Cm::SpatialVectorF* jointDofISInvStISW, 
	 PxReal* jointDofMinusStZExtW, PxReal* jointDofQStZIntIcW,
	 Cm::SpatialVectorF& deltaZAExtParent, Cm::SpatialVectorF& deltaZAIntIcParent)
	{
		deltaZAExtParent = linkZExtW;
		deltaZAIntIcParent = linkZIntIcW;

		//	The goal is to propagate the articulated z.a force of a child link to the articulated z.a. force of its parent link.
		//	We will compute a term that can be added to the articulated z.a. force of the parent link. 

		//	This function only references the child link.  
		//	Mirtich uses the notation i for the child and i-1 for the parent.
		//	We have a more general configuration that allows a parent to have multiple children but in what follows "i" shall refer to the 
		//	child and "i-1" to the parent.

		// Another goal is to propagate the articulated spatial inertia from the child link to the parent link.
		// We will compute a term that can be added to the articulated spatial inertia of the parent link.
		// The Mirtich equivalent is:
		//	I_i^A - [I_i^A * s_i^T *Inv(s_i^T *I_i^A * s_i) * s_i^T * I_i^A]

		//The term that is to be added to the parent link has the Mirtich formulation:
		//	Delta_Z_i-1 = (Z_i^A + I_i^A * c_i) +  [I_i^A * s_i]*[Q_i - s_i^T * (Z_i^A + I_i^A * c_i)]/[s_i^T * I_i^A * s_i]

		//We do not have a single articulated z.a. force as outlined in Mirtich.
		//Instead we have a term that accounts for external forces and a term that accounts for internal forces.

		//We can generalise the Mirtich formulate to account for internal and external terms:
		//	Delta_ZExt_i-1 = ZAExt_i +  [I_i^A * s_i] * [-s_i^T * ZAExt_i]/[s_i^T * I_i^A * s_i]
		//	Delta_ZInt_i-1 = ZAInt_i + I_i^A * c_i + [I_i^A * s_i] * [Q_i - s_i^T * (ZAInt_i + I_i^A * c_i)]/[s_i^T * I_i^A * s_i]
		//	Delta_Z_i-1 = Delta_ZExt_i-1 + Delta_ZInt_i-1

		//We have function input arguments ZExt and ZIntIc. 
		//In Mirtich terms these are ZAExt_i and ZAInt_i + I_i^A * c_i.

		//Using the function arguments here we have:
		//	Delta_ZExt_i-1 = ZAExt + [I_i^A * s_i] * [-s_i^T * ZAExt]/[s_i^T * I_i^A * s_i]
		//	Delta_ZInt_i-1 = ZAIntIc + [I_i^A * s_i] * [Q_i - s_i^T * ZAIntIc]/[s_i^T * I_i^A * s_i]
		
		//Isn't it odd that we add Q_i to the internal term rather than the external term?

		SpatialMatrix spatialInertia;
		switch (jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		case PxArticulationJointType::eREVOLUTE:
		case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
		{
			const Cm::UnAlignedSpatialVector& sa = jointMotionMatricesW[0];

#if FEATHERSTONE_DEBUG
			PxVec3 u = (jointType == PxArticulationJointType::ePRISMATIC) ? sa.bottom : sa.top;

			PxMat33 armatureV = Outer(u, u) * jointTarget.armature[0];

			const PxReal armature =  u.dot(armatureV * u);
#endif

			const Cm::SpatialVectorF& Is = jointISW[0];

			//Mirtich equivalent: 1/[s_i^T * I_i^A * s_i]
			PxReal invStIS;
			{
				PxU32 dofId = dofIds[0];
				const PxReal stIs = (sa.innerProduct(Is) + jointTargetArmatures[dofId]);
				invStIS = ((stIs > 0.f) ? (1.f / stIs) : 0.f);
			}
			linkInvStISW.invStIs[0][0] = invStIS;

			//Mirtich equivalent: [I_i^A * s_i]/[s_i^T * I_i^A * s_i]
			const Cm::SpatialVectorF isID = Is * invStIS;
			jointDofISInvStISW[0] = isID;

			//(6x1)Is = [v0, v1]; (1x6)stI = [v1, v0]
			//Cm::SpatialVector stI1(Is1.angular, Is1.linear);
			const Cm::SpatialVectorF stI(Is.bottom, Is.top);

			//Mirtich equivalent: I_i^A * s_i^T *Inv(s_i^T *I_i^A * s_i) * s_i^T * I_i^A
			//Note we will compute I_i^A - [I_i^A * s_i^T *Inv(s_i^T *I_i^A * s_i) * s_i^T * I_i^A] later in the function.
			spatialInertia = SpatialMatrix::constructSpatialMatrix(isID, stI);

			//[I_i^A * s_i] * [-s_i^T * ZAExt]/[s_i^T * I_i^A * s_i]
			{
				const PxReal innerprod = sa.innerProduct(linkZExtW);
				const PxReal diff = -innerprod;
				jointDofMinusStZExtW[0] = diff;
				deltaZAExtParent += isID * diff;
			}

			//[I_i^A * s_i] * [Q_i - s_i^T * ZAIntIc]/[s_i^T * I_i^A * s_i]
			{
				const PxReal innerprod = sa.innerProduct(linkZIntIcW);
				const PxReal diff = (jointExternalForces ? jointExternalForces[0] : 0.0f) - innerprod;
				jointDofQStZIntIcW[0] = diff;
				deltaZAIntIcParent += isID * diff;
			}

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
			for (PxU32 ind = 0; ind < nbJointDofs; ++ind)
			{

#if FEATHERSTONE_DEBUG
				const Cm::UnAlignedSpatialVector& sa0 = motionMatrix[ind];
				const PxVec3 u = sa0.top;
				PxMat33 armatureV = Outer(u, u) * jointTarget.armature[ind];
				PxVec3 armatureU = armatureV * u;
#endif

				for (PxU32 ind2 = 0; ind2 < nbJointDofs; ++ind2)
				{
					const Cm::UnAlignedSpatialVector& sa = jointMotionMatricesW[ind2];

#if FEATHERSTONE_DEBUG
					const PxVec3 u1 = sa.top;

					const PxReal armature = u1.dot(armatureU);
#endif
				
					D[ind][ind2] = sa.innerProduct(jointISW[ind]);
				}
				PxU32 dofId = dofIds[ind];
				D[ind][ind] += jointTargetArmatures[dofId];
			}

			//PxMat33 invD = SpatialMatrix::invertSym33(D);
			const PxMat33 invD = D.getInverse();
			for (PxU32 ind = 0; ind < nbJointDofs; ++ind)
			{
				for (PxU32 ind2 = 0; ind2 < nbJointDofs; ++ind2)
				{
					linkInvStISW.invStIs[ind][ind2] = invD[ind][ind2];
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
			for (PxU32 ind = 0; ind < nbJointDofs; ++ind)
			{
				Cm::SpatialVectorF isID(PxVec3(0.f), PxVec3(0.f));
				const Cm::UnAlignedSpatialVector& sa = jointMotionMatricesW[ind];

				const PxReal stZ = sa.innerProduct(linkZExtW);
				const PxReal stZInt = sa.innerProduct(linkZIntIcW);

				//link.qstZIc[ind] = jF[ind] - stZ;
				const PxReal localQstZ = - stZ;
				const PxReal localQstZInt = (jointExternalForces ? jointExternalForces[ind] : 0.0f) -stZInt;
				jointDofMinusStZExtW[ind] = localQstZ;
				jointDofQStZIntIcW[ind] = localQstZInt;

				for (PxU32 ind2 = 0; ind2 < nbJointDofs; ++ind2)
				{
					const Cm::SpatialVectorF& Is = jointISW[ind2];
					isID += Is * invD[ind][ind2];
				}
				columns[0] += isID * jointISW[ind].bottom.x;
				columns[1] += isID * jointISW[ind].bottom.y;
				columns[2] += isID * jointISW[ind].bottom.z;
				columns[3] += isID * jointISW[ind].top.x;
				columns[4] += isID * jointISW[ind].top.y;
				columns[5] += isID * jointISW[ind].top.z;
				jointDofISInvStISW[ind] = isID;

				deltaZAExtParent += isID * localQstZ;
				deltaZAIntIcParent += isID * localQstZInt;

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
			return linkArticulatedInertiaW;
		}

		//(I - Is*Inv(sIs)*sI)
		spatialInertia = linkArticulatedInertiaW - spatialInertia;

		return spatialInertia;
	}

	SpatialMatrix FeatherstoneArticulation::computePropagateSpatialInertia_ZA_ZIc_NonSeparated
		(const PxArticulationJointType::Enum jointType, const PxU8 nbJointDofs, 
		 const Cm::UnAlignedSpatialVector* jointMotionMatrices, const Cm::SpatialVectorF* jointIs, 
		 const PxReal* jointTargetArmatures,
		 const PxU8* dofIds, const PxReal* jointExternalForces, // also take care of NULL here? when is this function used?
		 const SpatialMatrix& articulatedInertia, 
		 const Cm::SpatialVectorF& ZIc, 
		 InvStIs& invStIs, Cm::SpatialVectorF* isInvD, 
		 PxReal* qstZIc,
		 Cm::SpatialVectorF& deltaZParent)
	{
		deltaZParent = ZIc;

		//	The goal is to propagate the articulated z.a force of a child link to the articulated z.a. force of its parent link.
		//	We will compute a term that can be added to the articulated z.a. force of the parent link. 

		//	This function only references the child link.  
		//	Mirtich uses the notation i for the child and i-1 for the parent.
		//	We have a more general configuration that allows a parent to have multiple children but in what follows "i" shall refer to the 
		//	child and "i-1" to the parent.

		// Another goal is to propagate the articulated spatial inertia from the child link to the parent link.
		// We will compute a term that can be added to the articulated spatial inertia of the parent link.
		// The Mirtich equivalent is:
		//	I_i^A - [I_i^A * s_i^T *Inv(s_i^T *I_i^A * s_i) * s_i^T * I_i^A]

		//The term that is to be added to the parent link has the Mirtich formulation:
		//	Delta_Z_i-1 = (Z_i^A + I_i^A * c_i) +  [I_i^A * s_i]*[Q_i - s_i^T * (Z_i^A + I_i^A * c_i)]/[s_i^T * I_i^A * s_i]

		//We have function input arguments ZIntIc. 
		//In Mirtich terms this is: Z_i + I_i^A * c_i.

		//Using the function arguments here we have:
		//	Delta_Z_i-1 = ZIc + [I_i^A * s_i] * [Q_i - s_i^T * ZIc]/[s_i^T * I_i^A * s_i]
		
		SpatialMatrix spatialInertia;

		switch (jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		case PxArticulationJointType::eREVOLUTE:
		case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
		{
			const Cm::UnAlignedSpatialVector& sa = jointMotionMatrices[0];

#if FEATHERSTONE_DEBUG
			PxVec3 u = (jointType == PxArticulationJointType::ePRISMATIC) ? sa.bottom : sa.top;

			PxMat33 armatureV = Outer(u, u) * jointTarget.armature[0];

			const PxReal armature = u.dot(armatureV * u);
#endif

			const Cm::SpatialVectorF& Is = jointIs[0];

			//Mirtich equivalent: 1/[s_i^T * I_i^A * s_i]
			PxReal iStIs;
			{
				PxU32 dofId = dofIds[0];
				const PxReal stIs = (sa.innerProduct(Is) + jointTargetArmatures[dofId]);
				iStIs = (stIs > 1e-10f) ? (1.f / stIs) : 0.f;
			}
			invStIs.invStIs[0][0] = iStIs;

			//Mirtich equivalent: [I_i^A * s_i]/[s_i^T * I_i^A * s_i]
			Cm::SpatialVectorF isID = Is * iStIs;
			isInvD[0] = isID;

			//(6x1)Is = [v0, v1]; (1x6)stI = [v1, v0]
			//Cm::SpatialVector stI1(Is1.angular, Is1.linear);
			Cm::SpatialVectorF stI(Is.bottom, Is.top);

			//Mirtich equivalent: I_i^A * s_i^T *[1/(s_i^T *I_i^A * s_i)] * s_i^T * I_i^A
			//Note we will compute I_i^A - [I_i^A * s_i^T *[1/(s_i^T *I_i^A * s_i)] * s_i^T * I_i^A] later in the function.
			spatialInertia = SpatialMatrix::constructSpatialMatrix(isID, stI);

			//Mirtich equivalent: [I_i^A * s_i] * [-s_i^T * Z_i^A]/[s_i^T * I_i^A * s_i]
			{
				const PxReal innerProd = sa.innerProduct(ZIc);
				const PxReal diff = jointExternalForces[0] - innerProd;
				qstZIc[0] = diff;
				deltaZParent += isID * diff;
			}

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
			for (PxU32 ind = 0; ind < nbJointDofs; ++ind)
			{

#if FEATHERSTONE_DEBUG
				const Cm::UnAlignedSpatialVector& sa0 = motionMatrix[ind];
				const PxVec3 u = sa0.top;
				PxMat33 armatureV = Outer(u, u) * jointTarget.armature[ind];
				PxVec3 armatureU = armatureV * u;
#endif

				for (PxU32 ind2 = 0; ind2 < nbJointDofs; ++ind2)
				{
					const Cm::UnAlignedSpatialVector& sa = jointMotionMatrices[ind2];

#if FEATHERSTONE_DEBUG
					const PxVec3 u1 = sa.top;

					const PxReal armature = u1.dot(armatureU);
#endif

					D[ind][ind2] = sa.innerProduct(jointIs[ind]);
				}
				PxU32 dofId = dofIds[ind];
				D[ind][ind] += jointTargetArmatures[dofId];
				//D[ind][ind] *= 10.f;
			}

			PxMat33 invD = SpatialMatrix::invertSym33(D);
			for (PxU32 ind = 0; ind < nbJointDofs; ++ind)
			{
				for (PxU32 ind2 = 0; ind2 < nbJointDofs; ++ind2)
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
			for (PxU32 ind = 0; ind < nbJointDofs; ++ind)
			{
				Cm::SpatialVectorF isID(PxVec3(0.f), PxVec3(0.f));
				const Cm::UnAlignedSpatialVector& sa = jointMotionMatrices[ind];

				const PxReal stZ = sa.innerProduct(ZIc);

				//link.qstZIc[ind] = jF[ind] - stZ;
				const PxReal localQstZ = jointExternalForces[ind] - stZ;
				qstZIc[ind] = localQstZ;

				for (PxU32 ind2 = 0; ind2 < nbJointDofs; ++ind2)
				{
					const Cm::SpatialVectorF& Is = jointIs[ind2];
					isID += Is * invD[ind][ind2];
				}
				columns[0] += isID * jointIs[ind].bottom.x;
				columns[1] += isID * jointIs[ind].bottom.y;
				columns[2] += isID * jointIs[ind].bottom.z;
				columns[3] += isID * jointIs[ind].top.x;
				columns[4] += isID * jointIs[ind].top.y;
				columns[5] += isID * jointIs[ind].top.z;
				isInvD[ind] = isID;

				deltaZParent += isID * localQstZ;

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


	SpatialMatrix FeatherstoneArticulation::computePropagateSpatialInertia(
			const PxArticulationJointType::Enum jointType, const PxU8 nbDofs,
			const SpatialMatrix& articulatedInertia, const Cm::UnAlignedSpatialVector* motionMatrices,
			const Cm::SpatialVectorF* linkIs, 
			InvStIs& invStIs, Cm::SpatialVectorF* isInvD)
	{
		SpatialMatrix spatialInertia;

		switch (jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		case PxArticulationJointType::eREVOLUTE:
		case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
		{
			const Cm::UnAlignedSpatialVector& sa = motionMatrices[0];

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
			for (PxU8 ind = 0; ind < nbDofs; ++ind)
			{
				for (PxU8 ind2 = 0; ind2 < nbDofs; ++ind2)
				{
					const Cm::UnAlignedSpatialVector& sa = motionMatrices[ind2];
					D[ind][ind2] = sa.innerProduct(linkIs[ind]);
				}
			}

			PxMat33 invD = SpatialMatrix::invertSym33(D);
			for (PxU8 ind = 0; ind < nbDofs; ++ind)
			{
				for (PxU8 ind2 = 0; ind2 < nbDofs; ++ind2)
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
			for (PxU8 ind = 0; ind < nbDofs; ++ind)
			{
				Cm::SpatialVectorF isID(PxVec3(0.f), PxVec3(0.f));

				for (PxU8 ind2 = 0; ind2 < nbDofs; ++ind2)
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
	
	void FeatherstoneArticulation::computeArticulatedSpatialInertiaAndZ
		(const ArticulationLink* links, const PxU32 linkCount, const PxVec3* linkRsW,
		 const ArticulationJointCoreData* jointData,
		 const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW,
		 const Cm::SpatialVectorF* linkCoriolisVectors, const PxReal* jointDofForces,
		 Cm::SpatialVectorF* jointDofISW, InvStIs* linkInvStISW, Cm::SpatialVectorF* jointDofISInvStISW, PxReal* jointDofMinusStZExtW, PxReal* jointDofQStZIntIcW, 
		 Cm::SpatialVectorF* linkZAExtForcesW, Cm::SpatialVectorF* linkZAIntForcesW, SpatialMatrix* linkSpatialArticulatedInertiaW,
         SpatialMatrix& baseInvSpatialArticulatedInertiaW)
	{
		const PxU32 startIndex = PxU32(linkCount - 1);

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			const ArticulationLink& link = links[linkID];
			const ArticulationJointCoreData& jointDatum = jointData[linkID];
			const PxU32 jointOffset = jointDatum.jointOffset;
			const PxU8 nbDofs = jointDatum.nbDof;

			for (PxU8 ind = 0; ind < nbDofs; ++ind)
			{
				const Cm::UnAlignedSpatialVector tmp = linkSpatialArticulatedInertiaW[linkID] * jointDofMotionMatricesW[jointOffset + ind];
				jointDofISW[jointOffset + ind].top = tmp.top;
				jointDofISW[jointOffset + ind].bottom = tmp.bottom;
			}

			//Compute the terms to accumulate on the parent's articulated z.a force and articulated spatial inertia.
			Cm::SpatialVectorF deltaZAExtParent;
			Cm::SpatialVectorF deltaZAIntParent;
			SpatialMatrix spatialInertiaW;
			{
				const ArticulationJointCore* joint = link.inboundJoint;

				//calculate spatial zero acceleration force, this can move out of the loop
				const Cm::SpatialVectorF linkZW = linkZAExtForcesW[linkID];
				const Cm::SpatialVectorF linkIcW = linkSpatialArticulatedInertiaW[linkID] * linkCoriolisVectors[linkID];
				const Cm::SpatialVectorF linkZIntIcW = linkZAIntForcesW[linkID] + linkIcW;

				//(I - Is*Inv(sIs)*sI)
				//KS - we also bury Articulated ZA force and ZIc force computation in here because that saves
				//us some round-trips to memory!
				spatialInertiaW = 
					computePropagateSpatialInertia_ZA_ZIc(
						PxArticulationJointType::Enum(joint->jointType), jointDatum.nbDof, 
						&jointDofMotionMatricesW[jointOffset], &jointDofISW[jointOffset], 
						&joint->armature[0], &joint->dofIds[0], jointDofForces ? &jointDofForces[jointOffset] : NULL,
						linkSpatialArticulatedInertiaW[linkID], 
						linkZW, linkZIntIcW, 
						linkInvStISW[linkID], &jointDofISInvStISW[jointOffset],
						&jointDofMinusStZExtW[jointOffset], &jointDofQStZIntIcW[jointOffset],
						deltaZAExtParent, deltaZAIntParent);
			}

			//Accumulate the spatial inertia on the parent link.
			{
				//transform spatial inertia into parent space
				FeatherstoneArticulation::translateInertia(constructSkewSymmetricMatrix(linkRsW[linkID]), spatialInertiaW);

				// Make sure we do not propagate up negative inertias around the principal inertial axes 
				// due to numerical rounding errors
				const PxReal minPropagatedInertia = 0.f;
				spatialInertiaW.bottomLeft.column0.x = PxMax(minPropagatedInertia, spatialInertiaW.bottomLeft.column0.x);
				spatialInertiaW.bottomLeft.column1.y = PxMax(minPropagatedInertia, spatialInertiaW.bottomLeft.column1.y);
				spatialInertiaW.bottomLeft.column2.z = PxMax(minPropagatedInertia, spatialInertiaW.bottomLeft.column2.z);

				linkSpatialArticulatedInertiaW[link.parent] += spatialInertiaW;
			}

			//Accumulate the articulated z.a force on the parent link.
			{
				const Cm::SpatialVectorF translatedZA = FeatherstoneArticulation::translateSpatialVector(linkRsW[linkID], deltaZAExtParent);
				const Cm::SpatialVectorF translatedZAInt = FeatherstoneArticulation::translateSpatialVector(linkRsW[linkID], deltaZAIntParent);
				linkZAExtForcesW[link.parent] += translatedZA;
				linkZAIntForcesW[link.parent] += translatedZAInt;
			}
		}

		//cache base link inverse spatial inertia
		linkSpatialArticulatedInertiaW[0].invertInertiaV(baseInvSpatialArticulatedInertiaW);
	}

	void FeatherstoneArticulation::computeArticulatedSpatialInertiaAndZ_NonSeparated(ArticulationData& data, ScratchData& scratchData)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();
		SpatialMatrix* PX_RESTRICT spatialArticulatedInertia = data.getWorldSpatialArticulatedInertia();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = data.getWorldMotionMatrix();
		Cm::SpatialVectorF* PX_RESTRICT Is = data.getIsW();
		
		InvStIs* PX_RESTRICT invStIs = data.getInvStIS();
		Cm::SpatialVectorF* PX_RESTRICT IsInvDW = data.getISInvStIS();
		PxReal* PX_RESTRICT qstZIc = data.getQstZIc();

		SpatialMatrix& baseInvSpatialArticulatedInertia = data.getBaseInvSpatialArticulatedInertiaW();

		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		const Cm::SpatialVectorF* PX_RESTRICT coriolisVectors = scratchData.coriolisVectors;
		Cm::SpatialVectorF* PX_RESTRICT articulatedZA = scratchData.spatialZAVectors;
		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			const ArticulationLink& link = links[linkID];
			const ArticulationJointCore* joint = link.inboundJoint; //todo: check for & and const
			const ArticulationJointCoreData& jointDatum = jointData[linkID];
			const PxU32 jointOffset = jointDatum.jointOffset;
			const PxU8 nbDofs = jointDatum.nbDof;

			for (PxU8 ind = 0; ind < nbDofs; ++ind)
			{
				const Cm::UnAlignedSpatialVector tmp = spatialArticulatedInertia[linkID] * motionMatrix[jointOffset + ind];
				Is[jointOffset + ind].top = tmp.top;
				Is[jointOffset + ind].bottom = tmp.bottom;
			}

			//calculate spatial zero acceleration force, this can move out of the loop
			Cm::SpatialVectorF deltaZParent;
			SpatialMatrix spatialInertiaW;
			{
				Cm::SpatialVectorF Ic = spatialArticulatedInertia[linkID] * coriolisVectors[linkID];
				Cm::SpatialVectorF Z = articulatedZA[linkID] + Ic;

				//(I - Is*Inv(sIs)*sI)
				//KS - we also bury Articulated ZA force and ZIc force computation in here because that saves
				//us some round-trips to memory!
				spatialInertiaW = computePropagateSpatialInertia_ZA_ZIc_NonSeparated(
					PxArticulationJointType::Enum(link.inboundJoint->jointType), jointDatum.nbDof, 
					&motionMatrix[jointDatum.jointOffset], &Is[jointDatum.jointOffset], 
					&joint->armature[0], &joint->dofIds[0],
					&scratchData.jointForces[jointDatum.jointOffset], // AD what's the difference between the scratch and the articulation
																	  // data?
					spatialArticulatedInertia[linkID], 
					Z,
					invStIs[linkID], &IsInvDW[jointDatum.jointOffset],
					&qstZIc[jointDatum.jointOffset],
					deltaZParent);
			}

			//transform spatial inertia into parent space
			FeatherstoneArticulation::translateInertia(constructSkewSymmetricMatrix(rw[linkID]), spatialInertiaW);
			spatialArticulatedInertia[link.parent] += spatialInertiaW;

			const Cm::SpatialVectorF translatedZA = FeatherstoneArticulation::translateSpatialVector(rw[linkID], deltaZParent);
			articulatedZA[link.parent] += translatedZA;
		}

		//cache base link inverse spatial inertia
		spatialArticulatedInertia[0].invertInertiaV(baseInvSpatialArticulatedInertia);
	}

	void FeatherstoneArticulation::computeArticulatedSpatialInertia(ArticulationData& data)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();
		SpatialMatrix* PX_RESTRICT spatialArticulatedInertia = data.getWorldSpatialArticulatedInertia();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = data.getWorldMotionMatrix();
		Cm::SpatialVectorF* PX_RESTRICT Is = data.getIsW();
		InvStIs* PX_RESTRICT invStIs = data.getInvStIS();
		Cm::SpatialVectorF* PX_RESTRICT IsInvDW = data.getISInvStIS();
		SpatialMatrix& baseInvSpatialArticulatedInertia = data.getBaseInvSpatialArticulatedInertiaW();
		const PxVec3* PX_RESTRICT rw = data.getRw();

		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			const ArticulationLink& link = links[linkID];
			const ArticulationJointCoreData& jointDatum = jointData[linkID];
			const PxU32 jointOffset = jointDatum.jointOffset;
			const PxU8 nbDofs = jointDatum.nbDof;

			for (PxU8 ind = 0; ind < nbDofs; ++ind)
			{
				const Cm::UnAlignedSpatialVector tmp = spatialArticulatedInertia[linkID] * motionMatrix[jointOffset + ind];
				Is[jointOffset + ind].top = tmp.top;
				Is[jointOffset + ind].bottom = tmp.bottom;
			}

			//(I - Is*Inv(sIs)*sI)
			SpatialMatrix spatialInertiaW = computePropagateSpatialInertia(
				PxArticulationJointType::Enum(link.inboundJoint->jointType),
				jointDatum.nbDof, spatialArticulatedInertia[linkID], &motionMatrix[jointOffset],
				&Is[jointOffset], 
				invStIs[linkID], &IsInvDW[jointOffset]);

			//transform spatial inertia into parent space
			FeatherstoneArticulation::translateInertia(constructSkewSymmetricMatrix(rw[linkID]), spatialInertiaW);

			spatialArticulatedInertia[link.parent] += spatialInertiaW;
		}

		//cache base link inverse spatial inertia
		spatialArticulatedInertia[0].invertInertiaV(baseInvSpatialArticulatedInertia);
	}

	void FeatherstoneArticulation::computeArticulatedResponseMatrix
	(const PxArticulationFlags& articulationFlags, const PxU32 linkCount, 
	 const ArticulationJointCoreData* jointData,
	 const SpatialMatrix& baseInvArticulatedInertiaW, 
	 const PxVec3* linkRsW, const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW,
     const Cm::SpatialVectorF* jointDofISW, const InvStIs* linkInvStISW, const Cm::SpatialVectorF* jointDofIsInvDW, 
     ArticulationLink* links, TestImpulseResponse* testImpulseResponsesW)
	{
		//PX_PROFILE_ZONE("ComputeResponseMatrix", 0);

		//We can work out impulse response vectors by propagating an impulse to the root link, then back down to the child link using existing data.
		//Alternatively, we can compute an impulse response matrix, which is a vector of 6x6 matrices, which can be multiplied by the impulse vector to
		//compute the response. This can be stored in world space, saving transforms. It can also be computed incrementally, meaning it should not be
		//dramatically more expensive than propagating the impulse for a single constraint. Furthermore, this will allow us to rapidly compute the 
		//impulse response with the TGS solver allowing us to improve quality of equality positional constraints by properly reflecting non-linear motion
		//of the articulation rather than approximating it with linear projections.

		//The input expected is a local-space impulse and the output is a local-space impulse response vector

		if (articulationFlags & PxArticulationFlag::eFIX_BASE)
		{
			//Fixed base, so response is zero
			PxMemZero(testImpulseResponsesW, sizeof(TestImpulseResponse));
		}
		else
		{
			//Compute impulse response matrix. Compute the impulse response of unit responses on all 6 axes...
			const PxMat33& bottomRight = baseInvArticulatedInertiaW.getBottomRight();
			testImpulseResponsesW[0].linkDeltaVTestImpulseResponses[0] = Cm::SpatialVectorF(baseInvArticulatedInertiaW.topLeft.column0, baseInvArticulatedInertiaW.bottomLeft.column0);
			testImpulseResponsesW[0].linkDeltaVTestImpulseResponses[1] = Cm::SpatialVectorF(baseInvArticulatedInertiaW.topLeft.column1, baseInvArticulatedInertiaW.bottomLeft.column1);
			testImpulseResponsesW[0].linkDeltaVTestImpulseResponses[2] = Cm::SpatialVectorF(baseInvArticulatedInertiaW.topLeft.column2, baseInvArticulatedInertiaW.bottomLeft.column2);
			testImpulseResponsesW[0].linkDeltaVTestImpulseResponses[3] = Cm::SpatialVectorF(baseInvArticulatedInertiaW.topRight.column0, bottomRight.column0);
			testImpulseResponsesW[0].linkDeltaVTestImpulseResponses[4] = Cm::SpatialVectorF(baseInvArticulatedInertiaW.topRight.column1, bottomRight.column1);
			testImpulseResponsesW[0].linkDeltaVTestImpulseResponses[5] = Cm::SpatialVectorF(baseInvArticulatedInertiaW.topRight.column2, bottomRight.column2);

			links[0].cfm *= PxMax(testImpulseResponsesW[0].linkDeltaVTestImpulseResponses[0].bottom.x, PxMax(testImpulseResponsesW[0].linkDeltaVTestImpulseResponses[1].bottom.y, testImpulseResponsesW[0].linkDeltaVTestImpulseResponses[2].bottom.z));
		}

		//We want to compute the effect of a test impulse applied to child link.
		//But to do that we need to apply the negative of that impulse to the parent link.
		//We directly store the impulses that will be propagated to parent link to 
		//save negating later.
		const Cm::SpatialVectorF testLinkImpulses[6] = 
		{
			Cm::SpatialVectorF(PxVec3(-1, 0, 0), PxVec3(0, 0, 0)),
			Cm::SpatialVectorF(PxVec3(0, -1, 0), PxVec3(0, 0, 0)),
			Cm::SpatialVectorF(PxVec3(0, 0, -1), PxVec3(0, 0, 0)),
			Cm::SpatialVectorF(PxVec3(0, 0, 0), PxVec3(-1, 0, 0)),
			Cm::SpatialVectorF(PxVec3(0, 0, 0), PxVec3(0, -1, 0)),
			Cm::SpatialVectorF(PxVec3(0, 0, 0), PxVec3(0, 0, -1))
		};
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const PxVec3& parentLinkToChildLink = linkRsW[linkID];		//childLinkPos - parentLinkPos
			const PxU32 jointOffset = jointData[linkID].jointOffset;
			const PxU8 dofCount = jointData[linkID].nbDof;
			const PxU32 parentLinkId = links[linkID].parent;

			for (PxU32 i = 0; i < 6; ++i)
			{
				const Cm::SpatialVectorF& testLinkImpulse = testLinkImpulses[i];

				//(1) Propagate child link impulse (and zero joint impulse) to parent
				//Note: the impulse has already been negated.
				PxReal QMinusStZ[3] = { 0.f, 0.f, 0.f };
				const Cm::SpatialVectorF Zp = propagateImpulseW(
					parentLinkToChildLink,
					testLinkImpulse,
					NULL, &jointDofIsInvDW[jointOffset], &jointDofMotionMatricesW[jointOffset], dofCount,
					QMinusStZ);

				//(2) Get deltaV response for parent
				const Cm::SpatialVectorF deltaVParent = -testImpulseResponsesW[parentLinkId].getLinkDeltaVImpulseResponse(Zp);

				//(3) Propagate deltaV to child and apply test impulse (encoded in QMinusStZ).
				const Cm::SpatialVectorF deltaVChild = 
					propagateAccelerationW(
						parentLinkToChildLink, deltaVParent, 
						linkInvStISW[linkID], &jointDofMotionMatricesW[jointOffset], &jointDofISW[jointOffset], QMinusStZ, dofCount, 
						NULL);

				testImpulseResponsesW[linkID].linkDeltaVTestImpulseResponses[i] = deltaVChild;
			}

			links[linkID].cfm *= PxMax(testImpulseResponsesW[linkID].linkDeltaVTestImpulseResponses[0].bottom.x, PxMax(testImpulseResponsesW[linkID].linkDeltaVTestImpulseResponses[1].bottom.y, testImpulseResponsesW[linkID].linkDeltaVTestImpulseResponses[2].bottom.z));
		}
	}

	void FeatherstoneArticulation::computeArticulatedSpatialZ(ArticulationData& data, ScratchData& scratchData)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();

		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		Cm::SpatialVectorF* PX_RESTRICT coriolisVectors = scratchData.coriolisVectors;
		Cm::SpatialVectorF* PX_RESTRICT articulatedZA = scratchData.spatialZAVectors;

		PxReal* PX_RESTRICT jointForces = scratchData.jointForces;
		
		const SpatialMatrix* PX_RESTRICT worldSpatialArticulatedInertia = data.getWorldSpatialArticulatedInertia();

		PxReal* PX_RESTRICT qstZIc = data.getQstZIc();	// PT: TODO: unclear if the code meant getQstZIc() or getMinusStZExt()
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
		const Cm::SpatialVectorF* PX_RESTRICT iSInvStIS = data.getISInvStIS();
		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			const ArticulationLink& link = links[linkID];
		
			const ArticulationJointCoreData& jointDatum = jointData[linkID];

			//calculate spatial zero acceleration force, this can move out of the loop
			Cm::SpatialVectorF Ic = worldSpatialArticulatedInertia[linkID] * coriolisVectors[linkID];
			Cm::SpatialVectorF ZIc = articulatedZA[linkID] + Ic;

			const PxReal* jF = &jointForces[jointDatum.jointOffset];
			
			Cm::SpatialVectorF ZA = ZIc;
			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				const Cm::UnAlignedSpatialVector& sa = worldMotionMatrix[jointDatum.jointOffset + ind];
				const PxReal stZ = sa.innerProduct(ZIc);

				//link.qstZIc[ind] = jF[ind] - stZ;
				const PxReal qstZic = jF[ind] - stZ;
				qstZIc[jointDatum.jointOffset + ind] = qstZic;
				PX_ASSERT(PxIsFinite(qstZic));

				ZA += iSInvStIS[jointDatum.jointOffset + ind] * qstZic;
			}
			//accumulate childen's articulated zero acceleration force to parent's articulated zero acceleration
			articulatedZA[link.parent] += FeatherstoneArticulation::translateSpatialVector(rw[linkID], ZA);
		}
	}

	void FeatherstoneArticulation::computeJointAccelerationW(const PxU8 nbJointDofs,
		const Cm::SpatialVectorF& parentMotionAcceleration, const Cm::SpatialVectorF* jointDofISW, const InvStIs& linkInvStISW,
		const PxReal* jointDofQStZIcW,
		PxReal* jointAcceleration)
	{
		PxReal tJAccel[6];
		//Mirtich equivalent: Q_i - (s_i^T * I_i^A * a_i-1) - s_i^T * (Z_i^A + I_i^A * c_i)
		for (PxU32 ind = 0; ind < nbJointDofs; ++ind)
		{
			//stI * pAcceleration
			const PxReal temp = jointDofISW[ind].innerProduct(parentMotionAcceleration);
			tJAccel[ind] = (jointDofQStZIcW[ind] - temp);
		}

		//calculate jointAcceleration
		//Mirtich equivalent: [Q_i - (s_i^T * I_i^A * a_i-1) - s_i^T * (Z_i^A + I_i^A * c_i)]/[s_i^T * I_i^A * s_i]
		for (PxU32 ind = 0; ind < nbJointDofs; ++ind)
		{
			jointAcceleration[ind] = 0.f;
			for (PxU32 ind2 = 0; ind2 < nbJointDofs; ++ind2)
			{
				jointAcceleration[ind] += linkInvStISW.invStIs[ind2][ind] * tJAccel[ind2];
			}
			//PX_ASSERT(PxAbs(jointAcceleration[ind]) < 5000);
		}
	}
	
	void FeatherstoneArticulation::computeLinkAcceleration
	(const bool doIC, const PxReal dt,
	 const bool fixBase,
	 const ArticulationLink* links, const PxU32 linkCount, const ArticulationJointCoreData* jointDatas,
	 const Cm::SpatialVectorF* linkSpatialZAForces, const Cm::SpatialVectorF* linkCoriolisForces, const PxVec3* linkRws, 
	 const Cm::UnAlignedSpatialVector* jointDofMotionMatrices,
	 const SpatialMatrix& baseInvSpatialArticulatedInertiaW,
	 const InvStIs* linkInvStIs, 
	 const Cm::SpatialVectorF* jointDofIsWs, const PxReal* jointDofQstZics,
	 Cm::SpatialVectorF* linkMotionAccelerations, Cm::SpatialVectorF* linkMotionVelocities,
	 PxReal* jointDofAccelerations, PxReal* jointDofVelocities, PxReal* jointDofNewVelocities)
	{
		//we have initialized motionVelocity and motionAcceleration to be zero in the root link if
		//fix based flag is raised
		
		if (!fixBase)
		{
#if FEATHERSTONE_DEBUG
			SpatialMatrix result = invInertia * baseLinkDatum.spatialArticulatedInertia;

			bool isIdentity = result.isIdentity();

			PX_ASSERT(isIdentity);
			PX_UNUSED(isIdentity);
#endif
			//ArticulationLink& baseLink = data.getLink(0);
			//const PxTransform& body2World = baseLink.bodyCore->body2World;
			const Cm::SpatialVectorF accel = -(baseInvSpatialArticulatedInertiaW * linkSpatialZAForces[0]);
			linkMotionAccelerations[0] = accel;
			const Cm::SpatialVectorF deltaV = accel * dt;
			linkMotionVelocities[0] += deltaV;
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



		//printf("===========================\n");

		//calculate acceleration
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];

			const Cm::SpatialVectorF pMotionAcceleration = FeatherstoneArticulation::translateSpatialVector(-linkRws[linkID], linkMotionAccelerations[link.parent]);

			const ArticulationJointCoreData& jointDatum = jointDatas[linkID];
			const PxU32 jointOffset = jointDatum.jointOffset;
			const PxU32 nbDof = jointDatum.nbDof;

			//calculate jointAcceleration
			PxReal* jA = &jointDofAccelerations[jointOffset];
			const InvStIs& invStIs = linkInvStIs[linkID];
			computeJointAccelerationW(PxU8(nbDof), pMotionAcceleration, &jointDofIsWs[jointOffset], invStIs,
				&jointDofQstZics[jointOffset], jA);

			Cm::SpatialVectorF motionAcceleration = pMotionAcceleration;
			if (doIC)
				motionAcceleration += linkCoriolisForces[linkID];
			PxReal* jointVelocity = &jointDofVelocities[jointOffset];
			PxReal* jointNewVelocity = &jointDofNewVelocities[jointOffset];

			for (PxU32 ind = 0; ind < nbDof; ++ind)
			{
				const PxReal accel = jA[ind];
				const PxReal jVel = jointVelocity[ind] + accel * dt;
				jointVelocity[ind] = jVel;
				jointNewVelocity[ind] = jVel;
				motionAcceleration.top += jointDofMotionMatrices[jointOffset + ind].top * accel;
				motionAcceleration.bottom += jointDofMotionMatrices[jointOffset + ind].bottom * accel;
			}

			//KS - can we just work out velocities by projecting out the joint velocities instead of accumulating all this?
			linkMotionAccelerations[linkID] = motionAcceleration;
			PX_ASSERT(linkMotionAccelerations[linkID].isFinite());
			linkMotionVelocities[linkID] += motionAcceleration * dt;

			/*Cm::SpatialVectorF spatialForce = mArticulationData.mWorldSpatialArticulatedInertia[linkID] * motionAcceleration;

			Cm::SpatialVectorF zaForce = -spatialZAForces[linkID];

			int bob = 0;
			PX_UNUSED(bob);*/
		}
	}

	/**
	\brief Compute the contribution of a single rigid body to the moment of inertia of a collection
	of rigid bodies.
	\param[in] IiW is the moment of inertia of the ith rigid body in the world frame
	\param[in] mi is the mass of the ith rigid body
	\param[in] riMinusComW has value (ri - rcom) with ri denoting the position of the rigid body in the world frame
	and rcom the position of the centre of mass of the collection of rigid bodies in the world frame.
	*/	 
	static PxMat33 computeContributionToEnsembleMomentOfInertia(const PxMat33& IiW, const PxReal mi, const PxVec3& riMinusComW)
	{
		//We compute the 3x3 matrix R from riMinusComW and then return [IiW +  mi * (R^T * R)]
		//This can be computed in fewer steps than R^T * R because the outcome contains only a few repeated terms.
		const PxReal ax = riMinusComW.x;
		const PxReal ay = riMinusComW.y;
		const PxReal az = riMinusComW.z;
		const PxReal ax2 = ax*ax;
		const PxReal ay2 = ay*ay;
		const PxReal az2 = az*az;
		const PxReal negAxAy = -ax*ay;
		const PxReal negAxAz = -ax*az;
		const PxReal negAyAz = -ay*az;
		const PxVec3 col0(ay2 + az2, negAxAy, negAxAz);
		const PxVec3 col1(negAxAy, ax2 + az2, negAyAz);
		const PxVec3 col2(negAxAz, negAyAz, ax2 + ay2);
		const PxMat33 IContribution(IiW.column0 + col0*mi, IiW.column1 + col1*mi, IiW.column2 + col2*mi);
		return IContribution;
	}

	/**
	\brief Compute the linear and angular momentum of an articulation
	\param[in] rcomW is the position of the centre of mass of the articulation in the world frame 
	\param[in] recipMass is the reciprocal of the total mass of all links of the articulation
	\param[in] linkCount is the number of links of the articulation.
	\param[in] linkMasses is the mass of each link of the articulation
	\param[in] linkIsolatedSpatialArticulatedInertiasW is the isolated inertia in the world frame of each link of the articulation 
	\param[in] linkAccumulatedPosesW is the pose in the world frame of each link of the articulation
	\param[in] linkMotionVelocitiesW is the spatial velocity in the world frame of each link of the articulation
	\param[out] linMomentumW is the linear momentum in the world frame of the articulation
	\param[out] angMomentumW is the angular momentum in the world frame of the articulation
	\param[out] compoundInertiaW is an optional inertia in the world frame of the articulation
	\note linkMotionVelocitiesW must store the linear velocity of the ith link in linkMotionVelocitiesW[i].bottom 
	and the angular velocity of the ith link in linkMotionVelocitiesW[i].top	
	\note This is a template function that optionally computes the compound moment of inertia of the collection of
	rigid bodies.  compoundInertiaW must be non-NUll if computeCompoundInertia is true. compoundInertiaW
	may have any value if computeCompoundInertia is false because it will be ignored.
	*/
	template<bool computeCompoundInertia>
	static void computeMomentum(
		const PxVec3& rcomW, const PxReal recipMass,
		const PxU32 linkCount, 
		const PxReal* linkMasses, const PxMat33* linkIsolatedSpatialArticulatedInertiasW, 
		const PxTransform* linkAccumulatedPosesW, const Cm::SpatialVectorF* linkMotionVelocitiesW,
		PxVec3& linMomentumW, PxVec3& angMomentumW, PxMat33* compoundInertiaW)
	{
		//m_i is the mass of the ith rigid body
		//I_i is the moment of inertia of the ith rigid body
		//r_i is the position of the ith rigid body 
		//v_i is the linear velocity of the ith rigid body
		//w_i is the angular velocity of the ith rigid body
		//p_i is the linear momentum of the ith rigid body
		//M is the total mass of all bodies
		//rcom is the position of the centre of mass of the ensemble of bodies
		//vcom is the velocity of the centre of mass of the ensemble of bodies
		//We introduce rprime_i:  r_i = rcom + rprime_i
		//We introduce vprime_i:  v_i = vcom + vprime_i
		//Total angular momentum is
		//L =  sum { I_i w_i} + sum { r_i X p_i } 
		//We can rewrite the 2nd term above using rprime_i, vprime_i, m_i
		//L = + sum { I_i w_i} + sum { m_i * (rcom + rprime_i)  X (vcom + vprime_i) } 
		//The 2nd term above expands out to four terms. 
		//Two terms have the following form:
		//rcom X sum {m_i * vprime_i }
		//sum {m_i * rprime_i } X vcom
		//But sum {m_i * rprime_i } and sum {m_i * vprime_i } are 0 by definition.
		//We are left with 
		//L =  sum { I_i w_i} + sum { rprime_i X m_i * vprime_i} + M * (rcom X vcom)
		//Rewrite using r_i and rcom etc
		//L =  sum { I_i w_i} + sum { (r_i - rcom} X m_i * (v_i - vcom} + M * (rcom X vcom)
		//For reasons unknown we do not account for M * (rcom X vcom)

		//sum { m_i * v_i }
		linMomentumW = PxVec3(0.0f, 0.0f, 0.0f);
		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			const PxVec3& vi = linkMotionVelocitiesW[linkID].bottom;
			const PxReal mi = linkMasses[linkID];
			const PxVec3 pi = vi * mi;
			linMomentumW += pi;
		}

		//sum { I_i * w_i} + sum { m_i *(r_i - rcom) X (v_i - vcom)
		const PxVec3 vcomW = linMomentumW * recipMass;
		angMomentumW = PxVec3(0.0f, 0.0f, 0.0f);
		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			const PxReal mi = linkMasses[linkID];
			const PxMat33& Ii = linkIsolatedSpatialArticulatedInertiasW[linkID];
			const PxVec3& vi = linkMotionVelocitiesW[linkID].bottom;
			const PxVec3& wi = linkMotionVelocitiesW[linkID].top;
			const PxVec3& ri = linkAccumulatedPosesW[linkID].p;
			const PxVec3 riMinusrComW = ri - rcomW;
			const PxVec3 viMinusvComW = vi - vcomW; 
			const PxVec3 angMomi = (Ii * wi) + (riMinusrComW * mi).cross(viMinusvComW); //  + (rcom .cross(vcom))*mass
			angMomentumW += angMomi;

			if(computeCompoundInertia)	
				*compoundInertiaW += computeContributionToEnsembleMomentOfInertia(Ii, mi, riMinusrComW);
		}
	}			

	void FeatherstoneArticulation::computeLinkInternalAcceleration
	(	const PxReal dt,
		const bool fixBase,
		const PxVec3& rcom, const PxReal recipMass, const PxMat33* linkIsolatedSpatialArticulatedInertiasW,
		const SpatialMatrix& baseInvSpatialArticulatedInertiaW,
		const ArticulationLink* links, const PxU32 linkCount,
		const PxReal* linkMasses, const PxVec3* linkRsW, const PxTransform* linkAccumulatedPosesW,
		const Cm::SpatialVectorF* linkSpatialZAIntForcesW, const Cm::SpatialVectorF* linkCoriolisVectorsW,
		const ArticulationJointCoreData* jointDatas, const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW,
		const InvStIs* linkInvStISW, const Cm::SpatialVectorF* jointDofISW, const PxReal* jointDofQStZIntIcW,
		Cm::SpatialVectorF* linkMotionAccelerationsW, Cm::SpatialVectorF* linkMotionIntAccelerationsW, Cm::SpatialVectorF* linkMotionVelocitiesW,
		PxReal* jointDofAccelerations, PxReal* jointDofVelocities, PxReal* jointDofNewVelocities)
	{
		//we have initialized motionVelocity and motionAcceleration to be zero in the root link if
		//fix based flag is raised

		//We only attempt to conserve momentum if we have a floating base articulation.
		PxVec3 linMomentum0(PxZero);
		PxVec3 angMomentum0(PxZero);
		if(!fixBase)
		{		
			computeMomentum<false>(
				rcom, recipMass, 
				linkCount, linkMasses, linkIsolatedSpatialArticulatedInertiasW, linkAccumulatedPosesW, linkMotionVelocitiesW, 
				linMomentum0, angMomentum0, NULL);

#if FEATHERSTONE_DEBUG
			SpatialMatrix result = invInertia * baseLinkDatum.spatialArticulatedInertia;

			bool isIdentity = result.isIdentity();

			PX_ASSERT(isIdentity);
			PX_UNUSED(isIdentity);
#endif
			//ArticulationLink& baseLink = data.getLink(0);
			//const PxTransform& body2World = baseLink.bodyCore->body2World;
			const Cm::SpatialVectorF accel = -(baseInvSpatialArticulatedInertiaW * linkSpatialZAIntForcesW[0]);
			linkMotionIntAccelerationsW[0] = accel;
			linkMotionAccelerationsW[0] += accel;
			const Cm::SpatialVectorF deltaV = accel * dt;
			linkMotionVelocitiesW[0] += deltaV;
		}
		else
		{
			linkMotionIntAccelerationsW[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}


		//printf("===========================\n");

		//calculate acceleration
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];

			const Cm::SpatialVectorF pMotionAcceleration = FeatherstoneArticulation::translateSpatialVector(-linkRsW[linkID], linkMotionIntAccelerationsW[link.parent]);

			const ArticulationJointCoreData& jointDatum = jointDatas[linkID];
			const PxU32 jointOffset = jointDatum.jointOffset;
			const PxU32 nbDof = jointDatum.nbDof;

			//calculate jointAcceleration
			PxReal jIntAccel[DY_MAX_DOF];
			computeJointAccelerationW(PxU8(nbDof), pMotionAcceleration, &jointDofISW[jointOffset], linkInvStISW[linkID],
				&jointDofQStZIntIcW[jointOffset], jIntAccel);
			//printf("jA %f\n", jA[0]);

			//KS - TODO - separate integration of coriolis vectors!
			Cm::SpatialVectorF motionAcceleration = pMotionAcceleration + linkCoriolisVectorsW[linkID];
			PxReal* PX_RESTRICT jointVelocity = &jointDofVelocities[jointOffset];
			PxReal* PX_RESTRICT jointNewVelocity = &jointDofNewVelocities[jointOffset];

			PxReal* PX_RESTRICT jA = &jointDofAccelerations[jointOffset];
			for (PxU32 ind = 0; ind < nbDof; ++ind)
			{
				const PxReal accel = jIntAccel[ind];
				const PxReal jVel = jointVelocity[ind] + accel * dt;
				jointVelocity[ind] = jVel;
				jointNewVelocity[ind] = jVel;
				motionAcceleration.top += jointDofMotionMatricesW[jointOffset + ind].top * accel;
				motionAcceleration.bottom += jointDofMotionMatricesW[jointOffset + ind].bottom * accel;
				jA[ind] += accel;
			}

			//KS - can we just work out velocities by projecting out the joint velocities instead of accumulating all this?
			linkMotionIntAccelerationsW[linkID] = motionAcceleration;
			linkMotionAccelerationsW[linkID] += motionAcceleration;
			PX_ASSERT(linkMotionAccelerationsW[linkID].isFinite());
			const Cm::SpatialVectorF velDelta = (motionAcceleration) * dt;
			linkMotionVelocitiesW[linkID] += velDelta;
		}

		if (!fixBase)
		{
			PxVec3 linMomentum1(PxZero);
			PxVec3 angMomentum1(PxZero);
			PxMat33 compoundInertia(PxZero);
			computeMomentum<true>(
				rcom, recipMass,
				linkCount, linkMasses, linkIsolatedSpatialArticulatedInertiasW, linkAccumulatedPosesW, linkMotionVelocitiesW,
				linMomentum1, angMomentum1, &compoundInertia);

			const PxMat33 invCompoundInertia = compoundInertia.getInverse();

			//Compute the ratio of old angular momentum and new angular momentum.
			PxReal angRatio;
			{
				const PxReal numerator = angMomentum0.magnitude();
				const PxReal denominator = angMomentum1.magnitude();
				angRatio = denominator == 0.0f ? 1.0f : numerator / denominator;
			}
			
			//Compute the delta angular momentum from the ratio 
			//delta = (angMomentum1 * (angMomentum0/angMomentum1 - 1.0f) = angMomentum0 - angMomentum1
			const PxVec3 deltaAngMom = angMomentum1 * (angRatio - 1.0f);

			//Compute the delta angular velocity.
			const PxVec3 deltaAng = invCompoundInertia * deltaAngMom;

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				const PxVec3 offset = linkAccumulatedPosesW[linkID].p - rcom;
				const Cm::SpatialVectorF velChange(deltaAng, -offset.cross(deltaAng));
				linkMotionVelocitiesW[linkID] += velChange;
				const PxReal mass = linkMasses[linkID];
				linMomentum1 += velChange.bottom * mass;
			}

			const PxVec3 deltaLinMom = linMomentum0 - linMomentum1;
			const PxVec3 deltaLin = deltaLinMom * recipMass;

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				linkMotionVelocitiesW[linkID].bottom += deltaLin;
			}
		}
	}

	void FeatherstoneArticulation::computeJointTransmittedFrictionForce(const ArticulationData& data, Cm::SpatialVectorF* PX_RESTRICT transmittedForce)
	{
		//const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = data.getLinkCount() - 1;

		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (PxU32 linkID = startIndex; linkID > 1; --linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);

			//joint force transmitted from parent to child
			//transmittedForce[link.parent] += data.mChildToParent[linkID] *  transmittedForce[linkID];
			transmittedForce[link.parent] += FeatherstoneArticulation::translateSpatialVector(rw[linkID], transmittedForce[linkID]);
		}

		transmittedForce[0] = Cm::SpatialVectorF::Zero();
	}

	void FeatherstoneArticulation::computeUnconstrainedVelocities(FeatherstoneArticulation* articulation,
		PxReal dt, const PxVec3& gravity, PxReal invLengthScale, bool externalForcesEveryTgsIterationEnabled, bool tgs)
	{
		ArticulationData& data = articulation->mArticulationData;
		data.setDt(dt);

		// AD: would be nicer to just have a list of all dirty articulations and process that one.
		// but that means we need to add a task dependency before because we'll get problems with multithreading
		// if we don't process the same lists.
		if (articulation->mJcalcDirty) 
		{	
			articulation->mJcalcDirty = false;
			articulation->jcalc(data);
		}

		articulation->computeUnconstrainedVelocitiesInternal(gravity,
			invLengthScale, tgs ? externalForcesEveryTgsIterationEnabled : false);

		if(!tgs)
			articulation->setupSolverConstraints(data);
	}

	void FeatherstoneArticulation::updateArticulation(const PxVec3& gravity, const PxReal invLengthScale, const bool externalForcesEveryTgsIterationEnabled)
	{
		//Copy the link poses into a handy array.
		//Update the link separation vectors with the latest link poses.
		//Compute the motion matrices in the world frame using the latest link poses.
		{
			//constants
			const ArticulationLink* links = mArticulationData.getLinks();
			const PxU32 linkCount = mArticulationData.getLinkCount();
			const ArticulationJointCoreData* jointCoreDatas = mArticulationData.getJointData();
			const Cm::UnAlignedSpatialVector* jointDofMotionMatrices = mArticulationData.getMotionMatrix();

			//outputs
			PxTransform* linkAccumulatedPosesW = mArticulationData.getAccumulatedPoses();
			PxVec3* linkRsW = mArticulationData.getRw();
			Cm::UnAlignedSpatialVector* jointDofMotionMatricesW = mArticulationData.getWorldMotionMatrix();

			computeRelativeTransformC2P(
				links, linkCount, jointCoreDatas, jointDofMotionMatrices,
				linkAccumulatedPosesW, linkRsW, jointDofMotionMatricesW);
		}

		//computeLinkVelocities(mArticulationData, scratchData);
		{
			//constants
			const PxReal dt = mArticulationData.mDt;
			const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
			const PxU32 nbLinks = mArticulationData.mLinkCount;
			const PxTransform* linkAccumulatedPosesW = mArticulationData.getAccumulatedPoses();
			const PxVec3* linkRsW = mArticulationData.getRw();
			const ArticulationJointCoreData* jointCoreData = mArticulationData.mJointData;
			const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW = mArticulationData.getWorldMotionMatrix();

			//outputs
			Cm::SpatialVector* linkExternalAccelsW = mArticulationData.mExternalAcceleration;
			ArticulationLink* links = mArticulationData.mLinks;
			Cm::SpatialVectorF* linkMotionVelocitiesW = mArticulationData.getMotionVelocities();
			Cm::SpatialVectorF* linkMotionAccelerationsW = mArticulationData.getMotionAccelerations();
			Cm::SpatialVectorF* linkCoriolisVectorsW = mArticulationData.getCoriolisVectors();
			Cm::SpatialVectorF* linkZAExtForcesW = mArticulationData.getSpatialZAVectors();
			Cm::SpatialVectorF* linkZAIntForcesW = mArticulationData.getSpatialZAInternalVectors();
			Dy::SpatialMatrix* linkSpatialArticulatedInertiasW = mArticulationData.getWorldSpatialArticulatedInertia();
			PxMat33* linkIsolatedSpatialArticulatedInertiasW = mArticulationData.getWorldIsolatedSpatialArticulatedInertia();
			PxReal* linkMasses = mArticulationData.getMasses();
			PxReal* jointDofVelocities = mArticulationData.getJointVelocities();
			Cm::SpatialVectorF& rootPreMotionVelocityW = mArticulationData.mRootPreMotionVelocity;
			PxVec3& comW = mArticulationData.mCOM;
			PxReal& invMass = mArticulationData.mInvSumMass;

			computeLinkStates(
				dt, invLengthScale, gravity, fixBase,
				nbLinks,
				linkAccumulatedPosesW, linkExternalAccelsW, linkRsW, jointDofMotionMatricesW, jointCoreData, externalForcesEveryTgsIterationEnabled,
				links, linkMotionAccelerationsW, linkMotionVelocitiesW, linkZAExtForcesW, linkZAIntForcesW, linkCoriolisVectorsW,
				linkIsolatedSpatialArticulatedInertiasW, linkMasses, linkSpatialArticulatedInertiasW,
				jointDofVelocities,
				rootPreMotionVelocityW, comW, invMass);
		}

		{
			const PxU32 linkCount = mArticulationData.getLinkCount();
			if (linkCount > 1)
			{
				const Cm::SpatialVectorF* ZAForcesExtW = mArticulationData.getSpatialZAVectors();
				const Cm::SpatialVectorF* ZAForcesIntW = mArticulationData.getSpatialZAInternalVectors();
				Cm::SpatialVectorF* ZAForcesTransmittedW = mArticulationData.getTransmittedForces();
				for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
				{
					ZAForcesTransmittedW[linkID] = ZAForcesExtW[linkID] + ZAForcesIntW[linkID];
				}
			}
		}
		
		{	
			//Constant inputs.
			const ArticulationLink* links = mArticulationData.getLinks();
			const PxU32 linkCount = mArticulationData.getLinkCount();
			const PxVec3* linkRsW = mArticulationData.getRw();
			const ArticulationJointCoreData* jointData = mArticulationData.getJointData();
			const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW = mArticulationData.getWorldMotionMatrix();
			const Cm::SpatialVectorF* linkCoriolisVectorsW = mArticulationData.getCoriolisVectors();
			const PxReal* jointDofForces = externalForcesEveryTgsIterationEnabled ? NULL : mArticulationData.getJointForces();

			//Values that we need now and will cache for later use.	
			Cm::SpatialVectorF* jointDofISW = mArticulationData.getIsW();
			InvStIs* linkInvStISW = mArticulationData.getInvStIS();
			Cm::SpatialVectorF* jointDofISInvStIS = mArticulationData.getISInvStIS(); //[(I * s)/(s^T * I * s)
			PxReal* jointDofMinusStZExtW = mArticulationData.getMinusStZExt();		//[-s^t * ZExt]
			PxReal* jointDofQStZIntIcW = mArticulationData.getQStZIntIc();			//[Q - s^T*(ZInt + I*c)]

			//We need to compute these.
			Cm::SpatialVectorF* linkZAForcesExtW = mArticulationData.getSpatialZAVectors();
			Cm::SpatialVectorF* linkZAForcesIntW = mArticulationData.getSpatialZAInternalVectors();
			SpatialMatrix* linkSpatialInertiasW = mArticulationData.getWorldSpatialArticulatedInertia();
			SpatialMatrix& baseInvSpatialArticulatedInertiaW = mArticulationData.getBaseInvSpatialArticulatedInertiaW();
		
			computeArticulatedSpatialInertiaAndZ(
				links, linkCount, linkRsW,											//constants
				jointData,															//constants
				jointDofMotionMatricesW, linkCoriolisVectorsW, jointDofForces,		//constants
				jointDofISW, linkInvStISW, jointDofISInvStIS,						//compute and cache for later use
				jointDofMinusStZExtW, jointDofQStZIntIcW,							//compute and cache for later use
				linkZAForcesExtW, linkZAForcesIntW,									//outputs 
				linkSpatialInertiasW, baseInvSpatialArticulatedInertiaW);			//outputs
		}

		{
			//Constants
			const PxArticulationFlags& flags = mArticulationData.getArticulationFlags();
			const PxU32 linkCount = mArticulationData.getLinkCount();
			const ArticulationJointCoreData* jointData = mArticulationData.getJointData();
			const SpatialMatrix& baseInvSpatialArticulatedInertiaW = mArticulationData.getBaseInvSpatialArticulatedInertiaW();
			const PxVec3* linkRsW = mArticulationData.getRw();
			const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW = mArticulationData.getWorldMotionMatrix();
			const Cm::SpatialVectorF* jointDofISW = mArticulationData.getIsW();
			const InvStIs* linkInvStIsW = mArticulationData.getInvStIS();
			const Cm::SpatialVectorF* jointDofISInvDW = mArticulationData.getISInvStIS(); 

			//outputs
			ArticulationLink* links = mArticulationData.getLinks();
			TestImpulseResponse* linkImpulseResponseMatricesW = mArticulationData.getImpulseResponseMatrixWorld();

			computeArticulatedResponseMatrix(
				flags, linkCount,									//constants
				jointData, baseInvSpatialArticulatedInertiaW,		//constants
				linkRsW, jointDofMotionMatricesW,					//constants
				jointDofISW, linkInvStIsW, jointDofISInvDW, 		//constants
				links, linkImpulseResponseMatricesW);				//outputs
		}

		{
			//Constant terms.
			const bool doIC = false;
			const PxReal dt = mArticulationData.getDt();
			const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
			const ArticulationLink* links = mArticulationData.getLinks();
			const PxU32 linkCount = mArticulationData.getLinkCount();
			const ArticulationJointCoreData* jointDatas = mArticulationData.getJointData();
			const Cm::SpatialVectorF* linkSpatialZAForcesExtW = mArticulationData.getSpatialZAVectors();
			const Cm::SpatialVectorF* linkCoriolisForcesW = mArticulationData.getCoriolisVectors();
			const PxVec3* linkRsW = mArticulationData.getRw(); 
			const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW = mArticulationData.getWorldMotionMatrix();
			const SpatialMatrix& baseInvSpatialArticulatedInertiaW = mArticulationData.getBaseInvSpatialArticulatedInertiaW();

			//Cached constant terms.
			const InvStIs* linkInvStISW = mArticulationData.getInvStIS();
			const Cm::SpatialVectorF* jointDofISW = mArticulationData.getIsW();
			const PxReal* jointDofMinusStZExtW = mArticulationData.getMinusStZExt();	

			//Output
			Cm::SpatialVectorF* linkMotionVelocitiesW = mArticulationData.getMotionVelocities();
			Cm::SpatialVectorF* linkMotionAccelerationsW = mArticulationData.getMotionAccelerations();
			PxReal* jointDofAccelerations = mArticulationData.getJointAccelerations();
			PxReal* jointDofVelocities = mArticulationData.getJointVelocities();
			PxReal* jointDofNewVelocities = mArticulationData.getJointNewVelocities();

			computeLinkAcceleration(
					doIC, dt, 
					fixBase,
					links, linkCount, jointDatas,
					linkSpatialZAForcesExtW, linkCoriolisForcesW, linkRsW,
					jointDofMotionMatricesW, baseInvSpatialArticulatedInertiaW,
					linkInvStISW, jointDofISW, jointDofMinusStZExtW,
					linkMotionAccelerationsW, linkMotionVelocitiesW,
					jointDofAccelerations, jointDofVelocities, jointDofNewVelocities);
		}

		{
			//constants
			const PxReal dt = mArticulationData.getDt();
			const PxVec3& comW = mArticulationData.mCOM;
			const PxReal invSumMass = mArticulationData.mInvSumMass;
			const SpatialMatrix& baseInvSpatialArticulatedInertiaW = mArticulationData.mBaseInvSpatialArticulatedInertiaW;
			const ArticulationLink* links = mArticulationData.getLinks();
			const PxU32 linkCount = mArticulationData.getLinkCount();
			const ArticulationJointCoreData* jointDatas = mArticulationData.getJointData();
			const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
			const Cm::SpatialVectorF* linkSpatialZAIntForcesW = mArticulationData.getSpatialZAInternalVectors();
			const Cm::SpatialVectorF* linkCoriolisVectorsW = mArticulationData.getCoriolisVectors();
			const PxReal* linkMasses = mArticulationData.getMasses();
			const PxVec3* linkRsW = mArticulationData.getRw();
			const PxTransform* linkAccumulatedPosesW = mArticulationData.getAccumulatedPoses();
			const PxMat33* linkIsolatedSpatialArticulatedInertiasW = mArticulationData.getWorldIsolatedSpatialArticulatedInertia();
			const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW = mArticulationData.getWorldMotionMatrix();
			//cached data.
			const InvStIs* linkInvStISW = mArticulationData.getInvStIS();
			const Cm::SpatialVectorF* jointDofISW = mArticulationData.getIsW();
			const PxReal* jointDofQStZIntIcW = mArticulationData.getQStZIntIc();
			//output
			Cm::SpatialVectorF* linkMotionVelocitiesW = mArticulationData.getMotionVelocities();
			Cm::SpatialVectorF* linkMotionAccelerationsW = mArticulationData.getMotionAccelerations();
			Cm::SpatialVectorF* linkMotionAccelerationIntW = mArticulationData.getMotionAccelerationsInternal();
			PxReal* jointDofAccelerations = mArticulationData.getJointAccelerations();
			PxReal* jointVelocities = mArticulationData.getJointVelocities();
			PxReal* jointNewVelocities = mArticulationData.getJointNewVelocities();

			computeLinkInternalAcceleration(
				dt, fixBase, comW, invSumMass, linkIsolatedSpatialArticulatedInertiasW, baseInvSpatialArticulatedInertiaW,
				links, linkCount,
				linkMasses, linkRsW, linkAccumulatedPosesW,
				linkSpatialZAIntForcesW, linkCoriolisVectorsW,
				jointDatas, jointDofMotionMatricesW,
				linkInvStISW, jointDofISW, jointDofQStZIntIcW,
				linkMotionAccelerationsW, linkMotionAccelerationIntW, linkMotionVelocitiesW, 
				jointDofAccelerations, jointVelocities, jointNewVelocities);
		}

		{
			const PxU32 linkCount = mArticulationData.getLinkCount();

			Cm::SpatialVectorF* solverLinkSpatialDeltaVels = mArticulationData.getSolverLinkSpatialDeltaVels();
			PxMemZero(solverLinkSpatialDeltaVels, sizeof(Cm::SpatialVectorF) * linkCount);

			Cm::SpatialVectorF* solverLinkSpatialImpulses = mArticulationData.getSolverLinkSpatialImpulses();
			PxMemZero(solverLinkSpatialImpulses, sizeof(Cm::SpatialVectorF) * linkCount);
		}
	}


	void FeatherstoneArticulation::computeUnconstrainedVelocitiesInternal(
		const PxVec3& gravity, const PxReal invLengthScale, const bool externalForcesEveryTgsIterationEnabled)
	{
		//PX_PROFILE_ZONE("Articulations:computeUnconstrainedVelocities", 0);

		//mStaticConstraints.forceSize_Unsafe(0);
		mStatic1DConstraints.forceSize_Unsafe(0);
		mStaticContactConstraints.forceSize_Unsafe(0);

		PxMemZero(mArticulationData.mNbStatic1DConstraints, mArticulationData.mLinkCapacity * sizeof(PxU32));
		PxMemZero(mArticulationData.mNbStaticContactConstraints, mArticulationData.mLinkCapacity * sizeof(PxU32));
		//const PxU32 linkCount = mArticulationData.getLinkCount();

		mArticulationData.init();

		updateArticulation(gravity, invLengthScale, externalForcesEveryTgsIterationEnabled);
	
		if (mArticulationData.mLinkCount > 1)
		{
			//use individual zero acceleration force(we copy the initial Z value to the transmitted force buffers in initLink())
			const Cm::SpatialVectorF* motionAccelerations = mArticulationData.getMotionAccelerations();
			Cm::SpatialVectorF* spatialZAVectors = mArticulationData.getTransmittedForces();
			computeZAForceInv(mArticulationData, motionAccelerations, spatialZAVectors);
			computeJointTransmittedFrictionForce(mArticulationData, spatialZAVectors);
		}

		//the dirty flag is used in inverse dynamic
		mArticulationData.setDataDirty(true);

		//zero zero acceleration vector in the articulation data so that we can use this buffer to accumulated
		//impulse for the contacts/constraints in the PGS/TGS solvers
		//PxMemZero(mArticulationData.getSpatialZAVectors(), sizeof(Cm::SpatialVectorF) * linkCount);

		//Reset deferredQstZ and root deferredZ!
		PxMemZero(mArticulationData.getDeferredQstZ(), sizeof(PxReal) * mArticulationData.getDofs());
		mArticulationData.mRootDeferredZ = Cm::SpatialVectorF::Zero();

		// solver progress counters
		maxSolverNormalProgress = 0;
		maxSolverFrictionProgress = 0;
		solverProgress = 0;

		PxTransform* PX_RESTRICT preTransform = mArticulationData.getPreTransform();
		const ArticulationLink* PX_RESTRICT links = mArticulationData.getLinks();
		PxTransform* PX_RESTRICT accumulatedPoses = mArticulationData.getAccumulatedPoses();
		PxQuat* PX_RESTRICT deltaQ = mArticulationData.getDeltaQ();

		for (PxU32 a = 0; a < mArticulationData.getLinkCount(); ++a)
		{
			const PxTransform& body2World = links[a].bodyCore->body2World;
			accumulatedPoses[a] = body2World; // ?? this was already done in updateArticulation->computeRelativeTransformC2P
			preTransform[a] = body2World;
			deltaQ[a] = PxQuat(PxIdentity);
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
		if(jointRotation.w < 0.0f)
			jointRotation = -jointRotation;

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
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		const ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();

		PxReal* PX_RESTRICT jointPositions = data.getJointPositions();
		const PxQuat* PX_RESTRICT relativeQuats = data.getRelativeQuat();
		
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			const ArticulationJointCore* joint = link.inboundJoint;
			const ArticulationJointCoreData& jointDatum = jointData[linkID];
			PxReal* jPositions = &jointPositions[jointDatum.jointOffset];
			
			if (joint->jointType == PxArticulationJointType::eSPHERICAL)
			{
				const ArticulationLink& pLink = links[link.parent];
				//const PxTransform pBody2World = pLink.bodyCore->body2World;

				const PxU32 dof = jointDatum.nbDof;

				computeSphericalJointPositions(relativeQuats[linkID], link.bodyCore->body2World.q,
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
		}
	}

	void FeatherstoneArticulation::updateJointProperties(const PxReal* jointNewVelocities, PxReal* jointVelocities, PxReal* jointAccelerations)
	{
		const PxU32 dofs = mArticulationData.getDofs();
		const PxReal invDt = 1.f / mArticulationData.getDt();

		for (PxU32 i = 0; i < dofs; ++i)
		{
			const PxReal jNewVel = jointNewVelocities[i];
			const PxReal delta = jNewVel - jointVelocities[i];
			jointVelocities[i] = jNewVel;
			jointAccelerations[i] += delta * invDt;
		}
	}

	void FeatherstoneArticulation::propagateLinksDown(ArticulationData& data, const PxReal* jointVelocities, PxReal* jointPositions,
		Cm::SpatialVectorF* motionVelocities)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();

		const ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();

		const PxQuat* const PX_RESTRICT relativeQuats = data.getRelativeQuat();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = data.getMotionMatrix();

		const PxU32 linkCount = data.getLinkCount();

		const PxReal dt = data.getDt();
		
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];

			const ArticulationJointCoreData& jointDatum = jointData[linkID];
			//const PxTransform oldTransform = preTransforms[linkID];

			const ArticulationLink& pLink = links[link.parent];
			const PxTransform pBody2World = pLink.bodyCore->body2World;

			const ArticulationJointCore* joint = link.inboundJoint;

			const PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
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

				jPosition[0] = jPos;

				newParentToChild = relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				const PxVec3& u = motionMatrix[jointDatum.jointOffset].bottom;

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

				const PxVec3& u = motionMatrix[jointDatum.jointOffset].top;

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

				const PxVec3& u = motionMatrix[jointDatum.jointOffset].top;

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
					PxTransform* PX_RESTRICT accumulatedPoses = data.getAccumulatedPoses();
					const PxTransform oldTransform = accumulatedPoses[linkID];
#if 0
					Cm::SpatialVectorF worldVel = FeatherstoneArticulation::translateSpatialVector(-data.mRw[linkID], motionVelocities[link.parent]);
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
					//	const PxReal jVel = data.mMotionMatrix[jointDatum.jointOffset + i].top.dot(relVel);
					//	jVelocity[i] = jVel;
					//	/*const PxReal delta = jVel * dt;
					//	jPosition[i] += delta;*/
					//}
#endif

					//Gp and Gc are centre of mass poses of parent(p) and child(c) in the world frame.
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
					for (PxU32 i = 0; i < jointDatum.nbDof; ++i)
					{
						PxVec3 sa = data.getMotionMatrix(jointDatum.jointOffset + i).top;
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
				PX_ASSERT(false);
				break;
			}

			body2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
			body2World.p = pBody2World.p + body2World.q.rotate(r);

			PX_ASSERT(body2World.isSane());
			PX_ASSERT(body2World.isValid());	
		}
	}

	void FeatherstoneArticulation::updateBodies(FeatherstoneArticulation* articulation, Cm::SpatialVectorF* tempDeltaV, PxReal dt, bool integrateJointPositions)
	{		
		ArticulationData& data = articulation->mArticulationData;
		ArticulationLink* PX_RESTRICT links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		const PxReal* PX_RESTRICT masses = data.getMasses();
		Cm::SpatialVectorF* PX_RESTRICT motionVelocities = data.getMotionVelocities();
		Cm::SpatialVectorF* PX_RESTRICT posMotionVelocities = data.getPosIterMotionVelocities();

		Cm::SpatialVector* PX_RESTRICT externalAccels = data.getExternalAccelerations();
		Cm::SpatialVector zero = Cm::SpatialVector::zero();

		data.setDt(dt);

		//update joint velocities/accelerations due to contacts/constraints.
		if (data.mJointDirty)
		{
			//update delta joint velocity and motion velocity due to velocity iteration changes
			
			//update motionVelocities
			PxcFsFlushVelocity(*articulation, tempDeltaV);
		}

		Cm::SpatialVectorF momentum0 = Cm::SpatialVectorF::Zero();
		PxVec3 posMomentum(0.f);

		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		if (!fixBase)
		{
			const PxVec3 COM = data.mCOM;

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				const PxReal mass = masses[linkID];
				momentum0.top += motionVelocities[linkID].bottom * mass;
				posMomentum += posMotionVelocities[linkID].bottom * mass;
			}

			const PxVec3 rootVel = momentum0.top * data.mInvSumMass;

			const PxTransform* PX_RESTRICT preTransform = data.getPreTransform();
			const PxMat33* PX_RESTRICT worldIsolatedSpatialArticulatedInertia = data.getWorldIsolatedSpatialArticulatedInertia();

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				const PxReal mass = masses[linkID];
				const PxVec3 offsetMass = (preTransform[linkID].p - COM) * mass;
				const PxVec3 angMom = (worldIsolatedSpatialArticulatedInertia[linkID] * motionVelocities[linkID].top) +
					offsetMass.cross(motionVelocities[linkID].bottom - rootVel);
				momentum0.bottom += angMom;
			}
		}

		PxTransform* PX_RESTRICT accumulatedPoses = data.getAccumulatedPoses();
		if (!integrateJointPositions)
		{
			//TGS
			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				links[linkID].bodyCore->body2World = accumulatedPoses[linkID].getNormalized();
			}

			computeAndEnforceJointPositions(data);
		}
		else // PGS
		{
			if (!fixBase)
			{
				const PxTransform& preTrans = accumulatedPoses[0];

				const Cm::SpatialVectorF& posVel = data.getPosIterMotionVelocity(0);

				updateRootBody(posVel, preTrans, data, dt);
			}
			//using the original joint velocities and delta velocities changed in the positional iter to update joint position/body transform
			propagateLinksDown(data, data.getPosIterJointVelocities(), data.getJointPositions(), data.getPosIterMotionVelocities());
		}
		//Fix up momentum based on changes in pos. Only currently possible with non-fixed base

		if (!fixBase)
		{
			PxVec3* PX_RESTRICT linkRsW = data.getRw();
			const Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = data.getMotionMatrix();
			const PxReal* PX_RESTRICT jointNewVelocities = data.getJointNewVelocities();

			PxVec3 COM = data.mLinks[0].bodyCore->body2World.p * masses[0];
			accumulatedPoses[0] = data.mLinks[0].bodyCore->body2World;

			PxVec3 sumLinMom = motionVelocities[0].bottom * masses[0];
			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				PxU32 parent = data.mLinks[linkID].parent;

				const PxTransform childPose = data.mLinks[linkID].bodyCore->body2World;

				accumulatedPoses[linkID] = childPose;

				PxVec3 rw = childPose.p - accumulatedPoses[parent].p;

				linkRsW[linkID] = rw;

				ArticulationJointCoreData& jointDatum = data.mJointData[linkID];

				const PxReal* jVelocity = &jointNewVelocities[jointDatum.jointOffset];

				Cm::SpatialVectorF vel = FeatherstoneArticulation::translateSpatialVector(-rw, motionVelocities[parent]);
				Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
				for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
				{
					const PxReal jVel = jVelocity[ind];
					//deltaV += data.mWorldMotionMatrix[jointDatum.jointOffset + ind] * jVel;
					deltaV += motionMatrix[jointDatum.jointOffset + ind] * jVel;
				}

				vel.top += childPose.rotate(deltaV.top);
				vel.bottom += childPose.rotate(deltaV.bottom);

				motionVelocities[linkID] = vel;

				PxReal mass = masses[linkID];
				COM += childPose.p * mass;
				sumLinMom += vel.bottom * mass;
			}

			COM *= data.mInvSumMass;

			PxMat33 sumInertia(PxZero);

			PxVec3 sumAngMom(0.f);

			PxVec3 rootLinVel = sumLinMom * data.mInvSumMass;

#if PX_DEBUG
			PxMat33* PX_RESTRICT worldIsolatedSpatialArticulatedInertia = data.getWorldIsolatedSpatialArticulatedInertia();
#endif

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				const PxReal mass = masses[linkID];

				const PxVec3 offset = accumulatedPoses[linkID].p - COM;
				PxMat33 inertia;
				const PxMat33 R(accumulatedPoses[linkID].q);

				const PxVec3 inertiaTensor = Cm::safeRecip<PxVec3>(data.getLink(linkID).bodyCore->inverseInertia);

				const PxVec3 offsetMass = offset * mass;

				Cm::transformInertiaTensor(inertiaTensor, R, inertia);
				//Only needed for debug validation
#if PX_DEBUG
				worldIsolatedSpatialArticulatedInertia[linkID] = inertia;
#endif
				sumInertia += computeContributionToEnsembleMomentOfInertia(inertia, mass, offset);
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

			const PxTransform* PX_RESTRICT poses = data.getAccumulatedPoses();
			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				const PxVec3 offset = poses[linkID].p - COM;
				const Cm::SpatialVectorF velChange(angDelta, -offset.cross(angDelta));
				motionVelocities[linkID] += velChange;
				const PxReal mass = masses[linkID];
				sumLinMom += velChange.bottom * mass;
			}
#endif

			const PxVec3 linDelta = (momentum0.top - sumLinMom) * data.mInvSumMass;

			

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				motionVelocities[linkID].bottom += linDelta;
			}

			//if (integrateJointPositions)
			{
				const PxVec3 predictedCOM = data.mCOM + posMomentum * (data.mInvSumMass * dt);
				const PxVec3 posCorrection = predictedCOM - COM;

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
			const PxReal* jointNewVelocities = data.getJointNewVelocities();

			articulation->updateJointProperties(jointNewVelocities, jointVelocities, jointAccelerations);
		}

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
	}

	void FeatherstoneArticulation::updateRootBody(const Cm::SpatialVectorF& motionVelocity, 
		const PxTransform& preTransform, ArticulationData& data, const PxReal dt)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		//body2World store new body transform integrated from solver linear/angular velocity

		PX_ASSERT(motionVelocity.top.isFinite());
		PX_ASSERT(motionVelocity.bottom.isFinite());

		const ArticulationLink& baseLink = links[0];

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
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Articulation::getJointAcceleration() commonInit need to be called first to initialize data!");
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

		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;

		if (!fixBase)
		{
			const SpatialMatrix* worldSpatialArticulatedInertia = mArticulationData.getWorldSpatialArticulatedInertia();
			const SpatialMatrix inverseArticulatedInertia = worldSpatialArticulatedInertia[0].getInverse();
			motionAccelerations[0] = -(inverseArticulatedInertia * spatialZAForces[0]);
		}
#if FEATHERSTONE_DEBUG
		else
		{
			PX_ASSERT(isSpatialVectorZero(motionAccelerations[0]));
		}
#endif

		const InvStIs* PX_RESTRICT invStIS = mArticulationData.getInvStIS();
		const Cm::SpatialVectorF* PX_RESTRICT isW = mArticulationData.getIsW();
		const PxReal* PX_RESTRICT qstZIc = mArticulationData.getQstZIc();	// PT: TODO: unclear if the code meant getQstZIc() or getMinusStZExt()
		const PxVec3* PX_RESTRICT rw = mArticulationData.getRw();

		PxReal* jointAccelerations = cache.jointAcceleration;
		//calculate acceleration
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = mArticulationData.getLink(linkID);

			//SpatialTransform p2C = linkDatum.childToParent.getTranspose();
			//Cm::SpatialVectorF pMotionAcceleration = mArticulationData.mChildToParent[linkID].transposeTransform(motionAccelerations[link.parent]);

			Cm::SpatialVectorF pMotionAcceleration = FeatherstoneArticulation::translateSpatialVector(-rw[linkID], motionAccelerations[link.parent]);

			const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
			//calculate jointAcceleration
			PxReal* jA = &jointAccelerations[jointDatum.jointOffset];
			const InvStIs& invStIs = invStIS[linkID];
			computeJointAccelerationW(jointDatum.nbDof, pMotionAcceleration, &isW[jointDatum.jointOffset], invStIs,
				&qstZIc[jointDatum.jointOffset], jA);

			Cm::SpatialVectorF motionAcceleration(PxVec3(0.f), PxVec3(0.f));

			const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();

			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				motionAcceleration.top += worldMotionMatrix[jointDatum.jointOffset + ind].top * jA[ind];
				motionAcceleration.bottom += worldMotionMatrix[jointDatum.jointOffset + ind].bottom * jA[ind];
			}

			motionAccelerations[linkID] = pMotionAcceleration + coriolisVectors[linkID] + motionAcceleration;
			PX_ASSERT(motionAccelerations[linkID].isFinite());
		}

		allocator->free(tempMemory);
	}

}//namespace Dy
}
