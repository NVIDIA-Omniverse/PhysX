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

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScSoftBodyCore.h"
#include "ScFEMClothCore.h"

#include "ScPhysics.h"
#include "ScSoftBodySim.h"
#include "DySoftBody.h"
#include "GuTetrahedronMesh.h"
#include "GuBV4.h"
#include "geometry/PxTetrahedronMesh.h"


using namespace physx;

Sc::SoftBodyCore::SoftBodyCore() :
	ActorCore(PxActorType::eSOFTBODY, PxActorFlag::eVISUALIZATION, PX_DEFAULT_CLIENT, 0),
	mGpuMemStat(0)
{
	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();

	mCore.initialRotation = PxQuat(PxIdentity);


	mCore.sleepThreshold = 5e-5f * scale.speed * scale.speed;
	mCore.wakeCounter = Physics::sWakeCounterOnCreation;
	mCore.freezeThreshold = 5e-6f * scale.speed * scale.speed;
	mCore.maxPenBias = -1e32f;//-PX_MAX_F32;
	
	mCore.solverIterationCounts = (1 << 8) | 4;
	mCore.dirty = true;
	mCore.mFlags = PxSoftBodyFlags(0);

	mCore.mPositionInvMass = NULL;
	mCore.mRestPosition = NULL;

	mCore.mSimPositionInvMass = NULL;
	mCore.mSimVelocity = NULL;

	mCore.mKinematicTarget = NULL;
}


Sc::SoftBodyCore::~SoftBodyCore() { }


void Sc::SoftBodyCore::setMaterial(const PxU16 handle)
{
	mCore.setMaterial(handle);
	mCore.dirty = true;
}

void Sc::SoftBodyCore::clearMaterials()
{
	mCore.clearMaterials();
	mCore.dirty = true;
}

void Sc::SoftBodyCore::setFlags(PxSoftBodyFlags flags)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
	{
		const bool wasDisabledSelfCollision = mCore.mFlags & PxSoftBodyFlag::eDISABLE_SELF_COLLISION;
		const bool isDisabledSelfCollision = flags & PxSoftBodyFlag::eDISABLE_SELF_COLLISION;

		if (wasDisabledSelfCollision != isDisabledSelfCollision)
		{
			if (isDisabledSelfCollision)
				sim->disableSelfCollision();
			else
				sim->enableSelfCollision();
		}
	}

	mCore.mFlags = flags;
	mCore.dirty = true;
}


template <typename I>
void computeRestPoses(PxVec4* pInvMasses, PxMat33* tetraRestPoses, const I* const indices, const PxU32 nbTetra)
{
	for (PxU32 i = 0; i < nbTetra; ++i)
	{
		const PxU32 startIndex = i * 4;
		// calculate rest pose
		const PxVec3 x0 = pInvMasses[indices[startIndex + 0]].getXYZ();
		const PxVec3 x1 = pInvMasses[indices[startIndex + 1]].getXYZ();
		const PxVec3 x2 = pInvMasses[indices[startIndex + 2]].getXYZ();
		const PxVec3 x3 = pInvMasses[indices[startIndex + 3]].getXYZ();

		const PxVec3 u1 = x1 - x0;
		const PxVec3 u2 = x2 - x0;
		const PxVec3 u3 = x3 - x0;

		PxMat33 Q = PxMat33(u1, u2, u3);
		tetraRestPoses[i] = Q.getInverse();
#if PX_CHECKED
		const float det = Q.getDeterminant();

		if (fabsf(det) <= 1.e-9f)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "computeRestPoses(): Degenerate or inverted tetrahedron\n");
		}
#endif
	}
}

PxFEMParameters Sc::SoftBodyCore::getParameter() const
{
	return mCore.parameters;
}

void Sc::SoftBodyCore::setParameter(const PxFEMParameters parameter)
{
	mCore.parameters = parameter;
	mCore.dirty = true;
}

PxReal Sc::SoftBodyCore::getSleepThreshold() const
{
	return mCore.sleepThreshold;
}

void Sc::SoftBodyCore::setSleepThreshold(const PxReal v)
{
	mCore.sleepThreshold = v;
	mCore.dirty = true;
}

PxReal Sc::SoftBodyCore::getFreezeThreshold() const
{
	return mCore.freezeThreshold;
}

void Sc::SoftBodyCore::setFreezeThreshold(const PxReal v)
{
	mCore.freezeThreshold = v;
	mCore.dirty = true;
}

void Sc::SoftBodyCore::setSolverIterationCounts(const PxU16 c)
{
	mCore.solverIterationCounts = c;
	mCore.dirty = true;
}

PxReal Sc::SoftBodyCore::getWakeCounter() const
{
	return mCore.wakeCounter;
}

void Sc::SoftBodyCore::setWakeCounter(const PxReal v)
{
	mCore.wakeCounter = v;
	mCore.dirty = true;

	Sc::SoftBodySim* sim = getSim();
	if (sim)
	{
		sim->onSetWakeCounter();
	}

}

void Sc::SoftBodyCore::setWakeCounterInternal(const PxReal v)
{
	mCore.wakeCounter = v;
	mCore.dirty = true;

	Sc::SoftBodySim* sim = getSim();
	if (sim)
	{
		sim->onSetWakeCounter();
	}
}

bool Sc::SoftBodyCore::isSleeping() const
{
	Sc::SoftBodySim* sim = getSim();
	return sim ? sim->isSleeping() : (mCore.wakeCounter == 0.0f);
}

void Sc::SoftBodyCore::wakeUp(PxReal wakeCounter)
{
	mCore.wakeCounter = wakeCounter;
	mCore.dirty = true;
}

void Sc::SoftBodyCore::putToSleep()
{
	mCore.wakeCounter = 0.0f;
	mCore.dirty = true;
}

PxActor* Sc::SoftBodyCore::getPxActor() const
{
	return PxPointerOffset<PxActor*>(const_cast<SoftBodyCore*>(this), gOffsetTable.scCore2PxActor[getActorCoreType()]);
}

void Sc::SoftBodyCore::attachShapeCore(ShapeCore* shapeCore)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim) 
	{
		sim->attachShapeCore(shapeCore);
		mCore.dirty = true;
	}
}

void Sc::SoftBodyCore::attachSimulationMesh(PxTetrahedronMesh* simulationMesh, PxSoftBodyAuxData* simulationState)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim) 
	{
		sim->attachSimulationMesh(simulationMesh, simulationState);
		mCore.dirty = true;
	}
}

Sc::SoftBodySim* Sc::SoftBodyCore::getSim() const
{
	return static_cast<Sc::SoftBodySim*>(ActorCore::getSim());
}


void Sc::SoftBodyCore::setSimulationFilterData(const PxFilterData& data)
{
	mFilterData = data;
}

PxFilterData Sc::SoftBodyCore::getSimulationFilterData() const
{
	return mFilterData;
}


void Sc::SoftBodyCore::addParticleFilter(Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
{
	Sc::SoftBodySim* sim = getSim();

	if (sim)
		sim->getScene().addParticleFilter(core, *sim, particleId, userBufferId, tetId);
}

void Sc::SoftBodyCore::removeParticleFilter(Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
{
	Sc::SoftBodySim* sim = getSim();

	if (sim)
		sim->getScene().removeParticleFilter(core, *sim, particleId, userBufferId, tetId);
}

PxU32 Sc::SoftBodyCore::addParticleAttachment(Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentric)
{
	Sc::SoftBodySim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addParticleAttachment(core, *sim, particleId, userBufferId, tetId, barycentric);

	return handle;
}

void Sc::SoftBodyCore::removeParticleAttachment(Sc::ParticleSystemCore* core, PxU32 handle)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeParticleAttachment(core, *sim, handle);
		setWakeCounter(ScInternalWakeCounterResetValue);
	}
}

void Sc::SoftBodyCore::addRigidFilter(Sc::BodyCore* core, PxU32 vertId)
{
	Sc::SoftBodySim* sim = getSim();

	if (sim)
		sim->getScene().addRigidFilter(core, *sim, vertId);

}

void Sc::SoftBodyCore::removeRigidFilter(Sc::BodyCore* core, PxU32 vertId)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
		sim->getScene().removeRigidFilter(core, *sim, vertId);
}

PxU32 Sc::SoftBodyCore::addRigidAttachment(Sc::BodyCore* core, PxU32 particleId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	Sc::SoftBodySim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if(sim)
		handle = sim->getScene().addRigidAttachment(core, *sim, particleId, actorSpacePose, constraint);

	return handle;
}

void Sc::SoftBodyCore::removeRigidAttachment(Sc::BodyCore* core, PxU32 handle)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeRigidAttachment(core, *sim, handle);
		setWakeCounter(ScInternalWakeCounterResetValue);
	}
}


void Sc::SoftBodyCore::addTetRigidFilter(Sc::BodyCore* core, PxU32 tetIdx)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
		sim->getScene().addTetRigidFilter(core, *sim, tetIdx);
}

void Sc::SoftBodyCore::removeTetRigidFilter(Sc::BodyCore* core, PxU32 tetIdx)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeTetRigidFilter(core, *sim, tetIdx);
	}
}
PxU32 Sc::SoftBodyCore::addTetRigidAttachment(Sc::BodyCore* core, PxU32 tetIdx, const PxVec4& barycentric, 
	const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	Sc::SoftBodySim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addTetRigidAttachment(core, *sim, tetIdx, barycentric, actorSpacePose, constraint);

	return handle;
}


void Sc::SoftBodyCore::addSoftBodyFilter(Sc::SoftBodyCore& core, PxU32 tetIdx0, PxU32 tetIdx1)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
		sim->getScene().addSoftBodyFilter(core, tetIdx0, *sim, tetIdx1);
}

void Sc::SoftBodyCore::removeSoftBodyFilter(Sc::SoftBodyCore& core, PxU32 tetIdx0, PxU32 tetIdx1)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
		sim->getScene().removeSoftBodyFilter(core, tetIdx0, *sim, tetIdx1);
}

void Sc::SoftBodyCore::addSoftBodyFilters(Sc::SoftBodyCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
		sim->getScene().addSoftBodyFilters(core, *sim, tetIndices0, tetIndices1, tetIndicesSize);
}

void Sc::SoftBodyCore::removeSoftBodyFilters(Sc::SoftBodyCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
		sim->getScene().removeSoftBodyFilters(core, *sim, tetIndices0, tetIndices1, tetIndicesSize);
}


PxU32 Sc::SoftBodyCore::addSoftBodyAttachment(Sc::SoftBodyCore& core, PxU32 tetIdx0, const PxVec4& triBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
	PxConeLimitedConstraint* constraint, PxReal constraintOffset)
{
	Sc::SoftBodySim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addSoftBodyAttachment(core, tetIdx0, triBarycentric0, *sim, tetIdx1, tetBarycentric1, constraint, constraintOffset);

	return handle;
}

void Sc::SoftBodyCore::removeSoftBodyAttachment(Sc::SoftBodyCore& core, PxU32 handle)
{
	Sc::SoftBodySim* sim = getSim();
	setWakeCounterInternal(ScInternalWakeCounterResetValue);
	core.setWakeCounterInternal(ScInternalWakeCounterResetValue);
	if (sim)
		sim->getScene().removeSoftBodyAttachment(core, *sim, handle);
}


void Sc::SoftBodyCore::addClothFilter(Sc::FEMClothCore& core, PxU32 triIdx, PxU32 tetIdx)
{
	Sc::SoftBodySim* sim = getSim();

	if (sim)
		sim->getScene().addClothFilter(core, triIdx, *sim, tetIdx);
}

void Sc::SoftBodyCore::removeClothFilter(Sc::FEMClothCore& core, PxU32 triIdx, PxU32 tetIdx)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
		sim->getScene().removeClothFilter(core, triIdx, *sim, tetIdx);
}

void Sc::SoftBodyCore::addVertClothFilter(Sc::FEMClothCore& core, PxU32 vertIdx, PxU32 tetIdx)
{
	Sc::SoftBodySim* sim = getSim();

	if (sim)
		sim->getScene().addVertClothFilter(core, vertIdx, *sim, tetIdx);
}

void Sc::SoftBodyCore::removeVertClothFilter(Sc::FEMClothCore& core, PxU32 vertIdx, PxU32 tetIdx)
{
	Sc::SoftBodySim* sim = getSim();
	if (sim)
		sim->getScene().removeVertClothFilter(core, vertIdx, *sim, tetIdx);
}

PxU32 Sc::SoftBodyCore::addClothAttachment(Sc::FEMClothCore& core, PxU32 triIdx, const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric,
	PxConeLimitedConstraint* constraint, PxReal constraintOffset)
{
	Sc::SoftBodySim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addClothAttachment(core, triIdx, triBarycentric, *sim, tetIdx, tetBarycentric, constraint, constraintOffset);

	return handle;
}

void Sc::SoftBodyCore::removeClothAttachment(Sc::FEMClothCore& core, PxU32 handle)
{
	Sc::SoftBodySim* sim = getSim();
	setWakeCounter(ScInternalWakeCounterResetValue);
	core.setWakeCounter(ScInternalWakeCounterResetValue);
	if (sim)
		sim->getScene().removeClothAttachment(core, *sim, handle);

}

PxU32 Sc::SoftBodyCore::getGpuSoftBodyIndex() const
{
	const Sc::SoftBodySim* sim = getSim();

	return sim ? sim->getGpuSoftBodyIndex() : 0xffffffff;
}

void Sc::SoftBodyCore::onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags)
{
	PX_UNUSED(shape);
	SoftBodySim* sim = getSim();
	if (!sim)
		return;
	SoftBodyShapeSim& s = sim->getShapeSim();

	if (notifyFlags & ShapeChangeNotifyFlag::eGEOMETRY)
		s.onVolumeOrTransformChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eMATERIAL)
		s.onMaterialChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eRESET_FILTERING)
		s.onResetFiltering();
	if (notifyFlags & ShapeChangeNotifyFlag::eSHAPE2BODY)
		s.onVolumeOrTransformChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eFILTERDATA)
		s.onFilterDataChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eCONTACTOFFSET)
		s.onContactOffsetChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eRESTOFFSET)
		s.onRestOffsetChange();
}

void Sc::SoftBodyCore::setKinematicTargets(const PxVec4* positions, PxSoftBodyFlags flags)
{
	mCore.mKinematicTarget = positions;

	if (positions != NULL)
	{
		mCore.mFlags |= PxSoftBodyFlags(flags & PxSoftBodyFlags(PxSoftBodyFlag::eKINEMATIC | PxSoftBodyFlag::ePARTIALLY_KINEMATIC));
	}
	else
	{
		mCore.mFlags.clear(PxSoftBodyFlag::eKINEMATIC);
		mCore.mFlags.clear(PxSoftBodyFlag::ePARTIALLY_KINEMATIC);
	}

	mCore.dirty = true;
}



#endif //PX_SUPPORT_GPU_PHYSX

