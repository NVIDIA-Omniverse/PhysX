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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScFEMClothCore.h"

#include "ScPhysics.h"
#include "ScFEMClothSim.h"
#include "DyFEMCloth.h"
#include "GuTetrahedronMesh.h"
#include "GuBV4.h"
#include "geometry/PxTetrahedronMesh.h"
#include "cudamanager/PxCudaContextManager.h"

using namespace physx;

Sc::FEMClothCore::FEMClothCore() :
	ActorCore(PxActorType::eFEMCLOTH, PxActorFlag::eVISUALIZATION, PX_DEFAULT_CLIENT, 0),
	mGpuMemStat(0)
{
	mCore.solverIterationCounts = (1 << 8) | 4;
	mCore.dirty = true;
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	mCore.mFlags = PxFEMClothFlags(0);
#endif

	mCore.mPositionInvMass = NULL;
	mCore.mVelocity = NULL;
	mCore.mRestPosition = NULL;
	mCore.wakeCounter = Physics::sWakeCounterOnCreation;
}

Sc::FEMClothCore::~FEMClothCore() { }

PxFEMParameters Sc::FEMClothCore::getParameter() const
{
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	return mCore.parameters;
#else
	return PxFEMParameters();
#endif
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
void Sc::FEMClothCore::setParameter(const PxFEMParameters& parameter)
{
	mCore.parameters = parameter;
	mCore.dirty = true;
}
#else
void Sc::FEMClothCore::setParameter(const PxFEMParameters&)
{
	mCore.dirty = true;
}
#endif

void Sc::FEMClothCore::addRigidFilter(Sc::BodyCore* core, PxU32 vertId)
{
	Sc::FEMClothSim* sim = getSim();

	if (sim)
		sim->getScene().addRigidFilter(core, *sim, vertId);

}

void Sc::FEMClothCore::removeRigidFilter(Sc::BodyCore* core, PxU32 vertId)
{
	Sc::FEMClothSim* sim = getSim();
	if (sim)
		sim->getScene().removeRigidFilter(core, *sim, vertId);
}


PxU32 Sc::FEMClothCore::addRigidAttachment(Sc::BodyCore* core, PxU32 particleId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* params)
{
	Sc::FEMClothSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
	{
		handle = sim->getScene().addRigidAttachment(core, *sim, particleId, actorSpacePose, params);

	}

	return handle;
}

void Sc::FEMClothCore::removeRigidAttachment(Sc::BodyCore* core, PxU32 handle)
{
	Sc::FEMClothSim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeRigidAttachment(core, *sim, handle);
		setWakeCounter(ScInternalWakeCounterResetValue);
	}
}

void Sc::FEMClothCore::addTriRigidFilter(Sc::BodyCore* core, PxU32 triIdx)
{
	Sc::FEMClothSim* sim = getSim();
	
	if (sim)
		sim->getScene().addTriRigidFilter(core, *sim, triIdx);

}

void Sc::FEMClothCore::removeTriRigidFilter(Sc::BodyCore* core, PxU32 triIdx)
{
	Sc::FEMClothSim* sim = getSim();
	if (sim)
		sim->getScene().removeTriRigidFilter(core, *sim, triIdx);
}


PxU32 Sc::FEMClothCore::addTriRigidAttachment(Sc::BodyCore* core, PxU32 triIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	Sc::FEMClothSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addTriRigidAttachment(core, *sim, triIdx, barycentric, actorSpacePose, constraint);

	return handle;
}

void Sc::FEMClothCore::removeTriRigidAttachment(Sc::BodyCore* core, PxU32 handle)
{
	Sc::FEMClothSim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeTriRigidAttachment(core, *sim, handle);
		setWakeCounter(ScInternalWakeCounterResetValue);
	}
}

void Sc::FEMClothCore::addClothFilter(Sc::FEMClothCore* otherCore, PxU32 otherTriIdx, PxU32 triIdx)
{
	Sc::FEMClothSim* sim = getSim();
	if (sim)
		sim->getScene().addClothFilter(*otherCore, otherTriIdx, *sim, triIdx);
}

void Sc::FEMClothCore::removeClothFilter(Sc::FEMClothCore* otherCore, PxU32 otherTriIdx, PxU32 triIdx)
{
	Sc::FEMClothSim* sim = getSim();
	if (sim)
		sim->getScene().removeClothFilter(*otherCore, otherTriIdx, *sim, triIdx);
}

PxU32 Sc::FEMClothCore::addClothAttachment(Sc::FEMClothCore* otherCore, PxU32 otherTriIdx, const PxVec4& otherTriBarycentric, PxU32 triIdx, const PxVec4& triBarycentric)
{
	Sc::FEMClothSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addTriClothAttachment(*otherCore, otherTriIdx, otherTriBarycentric, *sim, triIdx, triBarycentric);

	return handle;
}

void Sc::FEMClothCore::removeClothAttachment(Sc::FEMClothCore* otherCore, PxU32 handle)
{
	Sc::FEMClothSim* sim = getSim();
	setWakeCounter(ScInternalWakeCounterResetValue);
	otherCore->setWakeCounter(ScInternalWakeCounterResetValue);
	if (sim)
		sim->getScene().removeTriClothAttachment(*otherCore, *sim, handle);
}

void Sc::FEMClothCore::setBendingScales(const PxReal* const bendingScales, PxU32 nbElements) 
{
	mCore.mBendingScales.assign(bendingScales, bendingScales + nbElements);
	mCore.dirty = true;
}

const PxReal* Sc::FEMClothCore::getBendingScales() const
{
	return mCore.mBendingScales.empty() ? NULL : mCore.mBendingScales.begin(); 
}

void Sc::FEMClothCore::setSolverIterationCounts(const PxU16 c)
{
	mCore.solverIterationCounts = c;
	mCore.dirty = true;
}

PxActor* Sc::FEMClothCore::getPxActor() const
{
	return PxPointerOffset<PxActor*>(const_cast<FEMClothCore*>(this), gOffsetTable.scCore2PxActor[getActorCoreType()]);
}

void Sc::FEMClothCore::attachShapeCore(ShapeCore* shapeCore)
{
	Sc::FEMClothSim* sim = getSim();
	if (sim)
		sim->attachShapeCore(shapeCore);
}

PxReal Sc::FEMClothCore::getWakeCounter() const
{
	return mCore.wakeCounter;
}

void Sc::FEMClothCore::setWakeCounter(const PxReal v)
{
	mCore.wakeCounter = v;
	mCore.dirty = true;

	Sc::FEMClothSim* sim = getSim();
	if (sim)
	{
		sim->onSetWakeCounter();
	}

}

void Sc::FEMClothCore::setWakeCounterInternal(const PxReal v)
{
	mCore.wakeCounter = v;
	mCore.dirty = true;

	Sc::FEMClothSim* sim = getSim();
	if (sim)
	{
		sim->onSetWakeCounter();
	}
}


Sc::FEMClothSim* Sc::FEMClothCore::getSim() const
{
	return static_cast<Sc::FEMClothSim*>(ActorCore::getSim());
}

void Sc::FEMClothCore::setSimulationFilterData(const PxFilterData& data)
{
	mFilterData = data;
}

PxFilterData Sc::FEMClothCore::getSimulationFilterData() const
{
	return mFilterData;
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
void Sc::FEMClothCore::setFlags(PxFEMClothFlags flags)
{
	mCore.mFlags = flags;
	mCore.dirty = true;
}
#endif


void Sc::FEMClothCore::onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags)
{
	PX_UNUSED(shape);
	FEMClothSim* sim = getSim();
	if (!sim)
		return;
	FEMClothShapeSim& s = sim->getShapeSim();

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

#endif //PX_SUPPORT_GPU_PHYSX

