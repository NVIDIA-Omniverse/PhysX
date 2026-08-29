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

#if PX_SUPPORT_OMNI_PVD
#include <stdio.h>

#include "foundation/PxPreprocessor.h"
#include "foundation/PxAllocator.h"
#include "common/PxProfileZone.h"

#include "ScIterators.h"

#include "omnipvd/NpOmniPvd.h"
#include "OmniPvdPxSampler.h"
#include "OmniPvdWriteStream.h"
#include "NpOmniPvdSetData.h"

#include "NpOmniPvdMetaData.h"


#include "NpPhysics.h"
#include "NpFactory.h"
#include "NpScene.h"
#include "NpAggregate.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpArticulationLink.h"
#include "NpArticulationMimicJoint.h"
#include "NpPBDParticleSystem.h"
#include "NpParticleBuffer.h"
#include "PxPBDMaterial.h"
#include "NpDeformableVolume.h"
#include "NpDeformableSurface.h"
#include "PxDeformableVolume.h"
#include "PxDeformableSurface.h"
#include "PxDeformableBody.h"
#include "PxDeformableSurfaceMaterial.h"
#include "NpShape.h"

using namespace physx;

// Forward declaration - defined later in this file
void streamTetMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxTetrahedronMesh& mesh);

class OmniPvdStreamContainer
{
public:
	OmniPvdStreamContainer();
	~OmniPvdStreamContainer();
	bool initOmniPvd();
	void registerClasses();
	bool dataWasWrittenSuccessfully();
	void setOmniPvdInstance(NpOmniPvd* omniPvdInstance);

	NpOmniPvd* mOmniPvdInstance;
	physx::PxMutex mMutex;
	OmniPvdPxCoreRegistrationData mRegistrationData;
};

class OmniPvdSamplerInternals : public physx::PxUserAllocated
{
public:
OmniPvdStreamContainer mPvdStream;
bool addSharedMeshIfNotSeen(const void* geom, OmniPvdSharedMeshEnum geomEnum); // Returns true if the Geom was not yet seen and added
physx::PxMutex mSampleMutex;
bool mIsSampling;

physx::PxMutex mSharedGeomsMutex;
physx::PxHashMap<const void*, OmniPvdSharedMeshEnum> mSharedMeshesMap;
};
OmniPvdSamplerInternals * samplerInternals = NULL;

namespace physx
{
NpOmniPvdSceneClient::NpOmniPvdSceneClient(physx::PxScene& scene) : mScene(scene), mFrameId(1)
{
}

NpOmniPvdSceneClient::~NpOmniPvdSceneClient()
{
}

void NpOmniPvdSceneClient::startFirstFrame(OmniPvdWriter& pvdWriter)
{
	pvdWriter.startFrame((OmniPvdContextHandle)(&mScene), mFrameId);
}

void NpOmniPvdSceneClient::resetFrameId()
{
	// Rewind to the first (odd = pre-sim) frame so a snapshot onto a freshly-bound
	// stream opens at frame 1, matching a fresh recording, regardless of how far
	// this scene's frame counter has already advanced on the previous stream.
	mFrameId = 1;
}

void NpOmniPvdSceneClient::incrementFrame(OmniPvdWriter& pvdWriter, bool recordProfileFrame)
{
	pvdWriter.stopFrame((OmniPvdContextHandle)(&mScene), mFrameId);
	mFrameId++;
	pvdWriter.startFrame((OmniPvdContextHandle)(&mScene), mFrameId);
	if (recordProfileFrame)
	{
		PX_PROFILE_FRAME("PVD", PxU64(&mScene));
	}
}

void NpOmniPvdSceneClient::stopLastFrame(OmniPvdWriter& pvdWriter)
{
	pvdWriter.stopFrame((OmniPvdContextHandle)(&mScene), mFrameId);
}

void NpOmniPvdSceneClient::addRigidDynamicForceReset(const physx::PxRigidDynamic* rigidDynamic)
{
	mResetRigidDynamicForce.insert(rigidDynamic);
}

void NpOmniPvdSceneClient::addRigidDynamicTorqueReset(const physx::PxRigidDynamic* rigidDynamic)
{
	mResetRigidDynamicTorque.insert(rigidDynamic);
}

void NpOmniPvdSceneClient::addRigidDynamicReset(const physx::PxRigidDynamic* rigidDynamic)
{
	mResetRigidDynamicForce.insert(rigidDynamic);
	mResetRigidDynamicTorque.insert(rigidDynamic);
}

void NpOmniPvdSceneClient::removeRigidDynamicReset(const physx::PxRigidDynamic* rigidDynamic)
{
	mResetRigidDynamicForce.erase(rigidDynamic);
	mResetRigidDynamicTorque.erase(rigidDynamic);
	PxVec3 zeroForce(0.0f, 0.0f, 0.0f);
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, force, *rigidDynamic, zeroForce);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, torque, *rigidDynamic, zeroForce);
	OMNI_PVD_WRITE_SCOPE_END
}

void NpOmniPvdSceneClient::addArticulationLinksForceReset(const PxArticulationReducedCoordinate* articulation)
{
	mResetArticulationLinksForce.insert(articulation);
}

void NpOmniPvdSceneClient::addArticulationLinksTorqueReset(const PxArticulationReducedCoordinate* articulation)
{
	mResetArticulationLinksTorque.insert(articulation);
}

void NpOmniPvdSceneClient::addArticulationJointsForceReset(const PxArticulationReducedCoordinate* articulation)
{
	mResetArticulationJointsForce.insert(articulation);
}

void NpOmniPvdSceneClient::addArticulationFromLinkFlagChangeReset(const physx::PxArticulationLink* link)
{
	PxArticulationReducedCoordinate& arti = link->getArticulation();
	{
		mResetArticulationLinksForce.insert(&arti);
		mResetArticulationLinksTorque.insert(&arti);
		mResetArticulationJointsForce.insert(&arti);
	}
}

#define SET_RIGID_BODY_ATTRIBS(resetRigidDynamic, rigiBodyAttribute, attribVal) \
{ \
	for(PxHashSet<const PxRigidDynamic*>::Iterator iter = resetRigidDynamic.getIterator(); !iter.done(); ++iter) \
	{ \
		const PxRigidDynamic* rdyn = *iter; \
		if (!(rdyn->getRigidBodyFlags() & PxRigidBodyFlag::eRETAIN_ACCELERATIONS)) \
		{ \
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigiBodyAttribute, *rdyn, attribVal); \
		} \
	} \
	resetRigidDynamic.clear(); \
}

#define SET_ARTICULATION_LINK_ATTRIBS(articulationHash, linkAttribute, attribVal) \
{ \
	for(PxHashSet<const PxArticulationReducedCoordinate*>::Iterator iter = articulationHash.getIterator(); !iter.done(); ++iter) \
	{ \
		const NpArticulationReducedCoordinate* npArticulation = static_cast<const NpArticulationReducedCoordinate*>(*iter);	\
		const PxU32 nbLinks = npArticulation->getNbLinks(); \
		const NpArticulationLink* const * npLinks = npArticulation->getLinks(); \
		for(PxU32 linkId = 0; linkId < nbLinks; linkId++) \
		{ \
			const PxRigidBody* pxBody = static_cast<const PxRigidBody*>(npLinks[linkId]); \
			if (!(pxBody->getRigidBodyFlags() & PxRigidBodyFlag::eRETAIN_ACCELERATIONS)) \
			{ \
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linkAttribute, *pxBody, attribVal); \
			} \
		} \
	} \
	articulationHash.clear();\
}

#define SET_SINGLE_ARTICULATION_LINK_ATTRIBS_NO_RETENTION(pxArticulation, linkAttribute, attribVal) \
{ \
	const NpArticulationReducedCoordinate* npArticulation = static_cast<const NpArticulationReducedCoordinate*>(pxArticulation);	\
	const PxU32 nbLinks = npArticulation->getNbLinks(); \
	const NpArticulationLink* const * npLinks = npArticulation->getLinks(); \
	for(PxU32 linkId = 0; linkId < nbLinks; linkId++) \
	{ \
		const PxRigidBody* pxBody = static_cast<const PxRigidBody*>(npLinks[linkId]); \
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linkAttribute, *pxBody, attribVal); \
	} \
}

void setSingleArticulationJointForces(const PxArticulationReducedCoordinate* pxArticulation, OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData,  const PxReal* dofForces)
{
	const NpArticulationReducedCoordinate* npArticulation = static_cast<const NpArticulationReducedCoordinate*>(pxArticulation);
	const PxU32 nbLinks = npArticulation->getNbLinks();
	const NpArticulationLink* const * npLinks = npArticulation->getLinks();
	for(PxU32 linkId = 0; linkId < nbLinks; linkId++)
	{
		const NpArticulationLink* npLink = npLinks[linkId];
		PxArticulationJointReducedCoordinate* pxJoint = npLink->getInboundJoint();
		if (pxJoint)
		{
			const PxU32 nbrDofs = npLink->getInboundJointDof();
			if (nbrDofs > 0)
			{
				OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointForce, *pxJoint, dofForces, nbrDofs);
			}
		}
	}
}

void NpOmniPvdSceneClient::removeArticulationReset(const PxArticulationReducedCoordinate* articulation)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		PxVec3 zeroForce(0.0f, 0.0f, 0.0f);
		const PxReal dofZeroForces[PxArticulationAxis::eCOUNT] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
		SET_SINGLE_ARTICULATION_LINK_ATTRIBS_NO_RETENTION(articulation, force, zeroForce)
		SET_SINGLE_ARTICULATION_LINK_ATTRIBS_NO_RETENTION(articulation, torque, zeroForce)
		setSingleArticulationJointForces(articulation, pvdWriter, pvdRegData, dofZeroForces);
	OMNI_PVD_WRITE_SCOPE_END

	mResetArticulationLinksForce.erase(articulation);
	mResetArticulationLinksTorque.erase(articulation);
	mResetArticulationJointsForce.erase(articulation);
}

void NpOmniPvdSceneClient::resetForces()
{
	if ( (mResetRigidDynamicForce.size() > 0) || (mResetRigidDynamicTorque.size() > 0) ||
		 (mResetArticulationLinksForce.size() > 0) || (mResetArticulationLinksTorque.size() > 0) || (mResetArticulationJointsForce.size() > 0)
	   )
	{
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

		PxVec3 zeroForce(0.0f, 0.0f, 0.0f);

		// RigidDynamic
		SET_RIGID_BODY_ATTRIBS(mResetRigidDynamicForce, force, zeroForce)
		SET_RIGID_BODY_ATTRIBS(mResetRigidDynamicTorque, torque, zeroForce)

		// Articulations
		SET_ARTICULATION_LINK_ATTRIBS(mResetArticulationLinksForce, force, zeroForce)
		SET_ARTICULATION_LINK_ATTRIBS(mResetArticulationLinksTorque, torque, zeroForce)

		// Articulation joints
		const PxReal dofZeroForces[PxArticulationAxis::eCOUNT] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
		for(PxHashSet<const PxArticulationReducedCoordinate*>::Iterator iter = mResetArticulationJointsForce.getIterator(); !iter.done(); ++iter)
		{
			setSingleArticulationJointForces(*iter, pvdWriter, pvdRegData, dofZeroForces);
		}
		mResetArticulationJointsForce.clear();

		OMNI_PVD_WRITE_SCOPE_END
	}
}

}

OmniPvdStreamContainer::OmniPvdStreamContainer()
{
	physx::PxMutex::ScopedLock myLock(mMutex);
	mOmniPvdInstance = NULL;
}

OmniPvdStreamContainer::~OmniPvdStreamContainer()
{
}

void OmniPvdStreamContainer::setOmniPvdInstance(NpOmniPvd* omniPvdInstance)
{
	mOmniPvdInstance = omniPvdInstance;
}

bool OmniPvdStreamContainer::initOmniPvd()
{
	physx::PxMutex::ScopedLock myLock(mMutex);

	registerClasses();

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OmniPvdObjectHandle metaDataInstanceHandle = reinterpret_cast<OmniPvdObjectHandle>(&mOmniPvdInstance->mMetaData);
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, metaDataInstanceHandle);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, physxVersionMajor,  metaDataInstanceHandle, mOmniPvdInstance->mMetaData.physxVersionMajor);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, physxVersionMinor,  metaDataInstanceHandle, mOmniPvdInstance->mMetaData.physxVersionMinor);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, physxVersionBugfix, metaDataInstanceHandle, mOmniPvdInstance->mMetaData.physxVersionBugfix);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, ovdIntegrationVersionMajor, metaDataInstanceHandle, mOmniPvdInstance->mMetaData.ovdIntegrationVersionMajor);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxOmniPvdMetaData, ovdIntegrationVersionMinor, metaDataInstanceHandle, mOmniPvdInstance->mMetaData.ovdIntegrationVersionMinor);

	PxPhysics& physicsRef = static_cast<PxPhysics&>(NpPhysics::getInstance());
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, physicsRef);
	const physx::PxTolerancesScale& tolScale = physicsRef.getTolerancesScale();
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tolerancesScale, physicsRef, tolScale);

	OMNI_PVD_WRITE_SCOPE_END

	return dataWasWrittenSuccessfully();
}

void OmniPvdStreamContainer::registerClasses()
{
	// Write the full class/attribute schema to the bound stream. startSampling() calls this once
	// per snapshot (through initOmniPvd) so every recording starts with its own schema block. The
	// writer's handle counters were zeroed by OmniPvdWriter::setWriteStream when the stream was
	// bound, so registerData() re-mints the same class/attribute handle sequence (registration is
	// deterministic), keeping the cached handles in mRegistrationData consistent.
	PxOmniPvd::ScopedExclusiveWriter writeLock(mOmniPvdInstance);
	OmniPvdWriter* writer = writeLock.getWriter();
	if (writer)
	{
		mRegistrationData.registerData(*writer);
	}
}

bool OmniPvdStreamContainer::dataWasWrittenSuccessfully()
{
	bool dataWasWrittenOk = false;
	PxOmniPvd::ScopedExclusiveWriter writeLock(mOmniPvdInstance);
	OmniPvdWriter* writer = writeLock.getWriter();
	if (writer)
	{
		uint32_t statusFlags = writer->getStatus();
		if (!(statusFlags & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE))
		{
			dataWasWrittenOk = true;
		}
	}
	return dataWasWrittenOk;
}


int streamStringLength(const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	#define OMNI_PVD_MAX_STRING_LENGTH 2048
	if (NpPhysics::getInstance().mOmniPvdSampler == NULL)
	{
		return 0;
	}
	if (name == NULL)
	{
		return 0;
	}
	int len = static_cast<int>(strnlen(name, OMNI_PVD_MAX_STRING_LENGTH));
	if (len > 0)
	{
		return len;
	}
	else
	{
		return 0;
	}
#else
	return 0;
#endif
}

// Explicit form for callers that already hold a write scope (snapshot / streamActorAttributes):
// it reuses the caller's writer instead of opening a nested one.
void streamActorName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxActor& a, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, name, a, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

// Standalone form for the public setName() API path, which holds no write scope of its own.
void streamActorName(const physx::PxActor & a, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamActorName(pvdWriter, pvdRegData, a, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

// Explicit forms for callers that already hold a write scope (the full-state snapshot emits the
// scene / articulation / joint name inside its scope). The standalone setName() API path uses the
// non-explicit overloads below, which open their own scope and delegate here.
void streamSceneName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxScene& s, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, name, s, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

void streamSceneName(const physx::PxScene & s, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamSceneName(pvdWriter, pvdRegData, s, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

void streamArticulationName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationReducedCoordinate& art, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, name, art, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

void streamArticulationName(const physx::PxArticulationReducedCoordinate & art, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamArticulationName(pvdWriter, pvdRegData, art, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

void streamArticulationJointName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationJointReducedCoordinate& joint, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, name, joint, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

void streamArticulationJointName(const physx::PxArticulationJointReducedCoordinate& joint, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamArticulationJointName(pvdWriter, pvdRegData, joint, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

// Explicit form for callers that already hold a write scope (snapshot / streamParticleBufferAttributes):
// it reuses the caller's writer instead of opening a nested one.
void streamParticleBufferName(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleBuffer& pb, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	int strLen = streamStringLength(name);
	if (strLen)
	{
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, name, pb, name, strLen + 1); // copies over the trailing zero too
	}
#endif
}

// Standalone form for the public setName() API path, which holds no write scope of its own.
void streamParticleBufferName(const physx::PxParticleBuffer& pb, const char* name)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamParticleBufferName(pvdWriter, pvdRegData, pb, name);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

void streamSphereGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxSphereGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxSphereGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxSphereGeometry, radius, g, g.radius);
}

void streamCapsuleGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxCapsuleGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, halfHeight, g, g.halfHeight);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, radius, g, g.radius);
}

void streamBoxGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxBoxGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxBoxGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxBoxGeometry, halfExtents, g, g.halfExtents);
}

void streamPlaneGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxPlaneGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPlaneGeometry, g);
}

void streamCustomGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxCustomGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometry, callbacks, g, g.callbacks);
}

void streamConvexCore(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxConvexCoreGeometry& g)
{
		switch (g.getCoreType())
		{
			case PxConvexCore::ePOINT:
			{
				const PxConvexCore::Point& c = g.getCore<PxConvexCore::Point>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCorePoint, OmniPvdObjectHandle(&c));
			}
			break;
			case PxConvexCore::eSEGMENT:
			{
				const PxConvexCore::Segment& c = g.getCore<PxConvexCore::Segment>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreSegment, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreSegment, length, OmniPvdObjectHandle(&c), c.length);
			}
			break;
			case PxConvexCore::eBOX:
			{
				const PxConvexCore::Box& c = g.getCore<PxConvexCore::Box>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreBox, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreBox, extents, OmniPvdObjectHandle(&c), c.extents);
			}
			break;
			case PxConvexCore::eELLIPSOID:
			{
				const PxConvexCore::Ellipsoid& c = g.getCore<PxConvexCore::Ellipsoid>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreEllipsoid, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreEllipsoid, radii, OmniPvdObjectHandle(&c), c.radii);
			}
			break;
			case PxConvexCore::eCYLINDER:
			{
				const PxConvexCore::Cylinder& c = g.getCore<PxConvexCore::Cylinder>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCylinder, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCylinder, height, OmniPvdObjectHandle(&c), c.height);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCylinder, radius, OmniPvdObjectHandle(&c), c.radius);
			}
			break;
			case PxConvexCore::eCONE:
			{
				const PxConvexCore::Cone& c = g.getCore<PxConvexCore::Cone>();
				OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCone, OmniPvdObjectHandle(&c));
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCone, height, OmniPvdObjectHandle(&c), c.height);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreCone, radius, OmniPvdObjectHandle(&c), c.radius);
			}
			break;
			default:
				break;
		}
}

void streamConvexCoreGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxConvexCoreGeometry& g)
{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreGeometry, g);
		streamConvexCore(pvdWriter, pvdRegData, g);
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreGeometry, core, g, g.getCoreData());
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexCoreGeometry, margin, g, g.getMargin());
}

void streamConvexMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxConvexMesh& mesh)
{
	if (samplerInternals->addSharedMeshIfNotSeen(&mesh, OmniPvdSharedMeshEnum::eOmniPvdConvexMesh))
	{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, mesh);

		const PxU32 nbPolys = mesh.getNbPolygons();
		const PxU8* polygons = mesh.getIndexBuffer();
		const PxVec3* verts = mesh.getVertices();
		const PxU32 nbrVerts = mesh.getNbVertices();

		PxU32 totalTris = 0;
		for (PxU32 i = 0; i < nbPolys; i++)
		{
			physx::PxHullPolygon data;
			mesh.getPolygonData(i, data);
			totalTris += data.mNbVerts - 2;
		}

		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
		PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(totalTris * 3), "tmpIndices");
		//TODO: this copy is useless

		PxU32 vertIndex = 0;
		for (PxU32 v = 0; v < nbrVerts; v++)
		{
			tmpVerts[vertIndex + 0] = verts[v].x;
			tmpVerts[vertIndex + 1] = verts[v].y;
			tmpVerts[vertIndex + 2] = verts[v].z;
			vertIndex += 3;
		}

		PxU32 triIndex = 0;
		for (PxU32 p = 0; p < nbPolys; p++)
		{
			physx::PxHullPolygon data;
			mesh.getPolygonData(p, data);
			PxU32 nbTris = data.mNbVerts - 2;
			const PxU32 vref0 = polygons[data.mIndexBase + 0 + 0];
			for (PxU32 t = 0; t < nbTris; t++)
			{
				const PxU32 vref1 = polygons[data.mIndexBase + t + 1];
				const PxU32 vref2 = polygons[data.mIndexBase + t + 2];
				tmpIndices[triIndex + 0] = vref0;
				tmpIndices[triIndex + 1] = vref1;
				tmpIndices[triIndex + 2] = vref2;
				triIndex += 3;
			}
		}

		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, verts, mesh, tmpVerts, 3 * nbrVerts);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, tris, mesh, tmpIndices, 3 * totalTris);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
	}
}

void streamConvexMeshGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxConvexMeshGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, scale, g, g.scale.scale);
	streamConvexMesh(pvdWriter, pvdRegData, *g.convexMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, convexMesh, g, g.convexMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, meshFlags, g, g.meshFlags);
}

void streamHeightField(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxHeightField& hf)
{
	if (samplerInternals->addSharedMeshIfNotSeen(&hf, OmniPvdSharedMeshEnum::eOmniPvdHeightField))
	{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightField, hf);
		const PxU32 nbCols = hf.getNbColumns();
		const PxU32 nbRows = hf.getNbRows();
		const PxU32 nbVerts = nbRows * nbCols;
		const PxU32 nbFaces = (nbCols - 1) * (nbRows - 1) * 2;
		physx::PxHeightFieldSample* sampleBuffer = (physx::PxHeightFieldSample*)PX_ALLOC(sizeof(physx::PxHeightFieldSample)*(nbVerts), "sampleBuffer");
		hf.saveCells(sampleBuffer, nbVerts * sizeof(physx::PxHeightFieldSample));
		//TODO: are the copies necessary?
		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbVerts * 3), "tmpVerts");
		PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(nbFaces * 3), "tmpIndices");
		for (PxU32 i = 0; i < nbRows; i++)
		{
			for (PxU32 j = 0; j < nbCols; j++)
			{
				const float x = PxReal(i);// *rs;
				const float y = PxReal(sampleBuffer[j + (i*nbCols)].height);// *hs;
				const float z = PxReal(j);// *cs;
				const PxU32 vertexIndex = 3 * (i * nbCols + j);
				float* vert = &tmpVerts[vertexIndex];
				vert[0] = x;
				vert[1] = y;
				vert[2] = z;
			}
		}
		for (PxU32 i = 0; i < (nbCols - 1); ++i)
		{
			for (PxU32 j = 0; j < (nbRows - 1); ++j)
			{
				PxU8 tessFlag = sampleBuffer[i + j * nbCols].tessFlag();
				PxU32 i0 = j * nbCols + i;
				PxU32 i1 = j * nbCols + i + 1;
				PxU32 i2 = (j + 1) * nbCols + i;
				PxU32 i3 = (j + 1) * nbCols + i + 1;
				// i2---i3
				// |    |
				// |    |
				// i0---i1
				// this is really a corner vertex index, not triangle index
				PxU32 mat0 = hf.getTriangleMaterialIndex((j*nbCols + i) * 2);
				PxU32 mat1 = hf.getTriangleMaterialIndex((j*nbCols + i) * 2 + 1);
				bool hole0 = (mat0 == PxHeightFieldMaterial::eHOLE);
				bool hole1 = (mat1 == PxHeightFieldMaterial::eHOLE);
				// first triangle
				tmpIndices[6 * (i * (nbRows - 1) + j) + 0] = hole0 ? i0 : i2; // duplicate i0 to make a hole
				tmpIndices[6 * (i * (nbRows - 1) + j) + 1] = i0;
				tmpIndices[6 * (i * (nbRows - 1) + j) + 2] = tessFlag ? i3 : i1;
				// second triangle
				tmpIndices[6 * (i * (nbRows - 1) + j) + 3] = hole1 ? i1 : i3; // duplicate i1 to make a hole
				tmpIndices[6 * (i * (nbRows - 1) + j) + 4] = tessFlag ? i0 : i2;
				tmpIndices[6 * (i * (nbRows - 1) + j) + 5] = i1;
			}
		}
		PX_FREE(sampleBuffer);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightField, verts, hf, tmpVerts, 3 * nbVerts);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightField, tris, hf, tmpIndices, 3 * nbFaces);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
	}
}

void streamHeightFieldGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxHeightFieldGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, g);

	PxVec3 vertScale(g.rowScale, g.heightScale, g.columnScale);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, scale, g, vertScale);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, heightField, g, g.heightField);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, meshFlags, g, g.heightFieldFlags);
}

void streamActorAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxActor& actor, const bool supportStandaloneBounds)
{
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, flags, actor, actor.getActorFlags());
	streamActorName(pvdWriter, pvdRegData, actor, actor.getName());
	// Should we stream the worldBounds if the actor is not part of a Scene yet?
	if (supportStandaloneBounds || actor.getScene())
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, actor, actor.getWorldBounds())
	}
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, dominance, actor, actor.getDominanceGroup())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, ownerClient, actor, actor.getOwnerClient())
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, environmentID, actor, actor.getEnvironmentID());
}

void streamRigidActorAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const PxRigidActor &ra)
{
	streamActorAttributes(pvdWriter, pvdRegData, ra, true);

	PxTransform t = ra.getGlobalPose();
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, ra, t)

	// Stream shapes too
	const int nbrShapes = ra.getNbShapes();
	for (int s = 0; s < nbrShapes; s++)
	{
		PxShape* shape[1];
		ra.getShapes(shape, 1, s);
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, shapes, ra, *shape[0])
	}
}

void streamRigidBodyAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxRigidBody& rigidBody)
{
	streamRigidActorAttributes(pvdWriter, pvdRegData, rigidBody);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, cMassLocalPose, rigidBody, rigidBody.getCMassLocalPose());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, mass, rigidBody, rigidBody.getMass());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, massSpaceInertiaTensor, rigidBody, rigidBody.getMassSpaceInertiaTensor());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearDamping, rigidBody, rigidBody.getLinearDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularDamping, rigidBody, rigidBody.getAngularDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, rigidBody, rigidBody.getLinearVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, rigidBody, rigidBody.getAngularVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxLinearVelocity, rigidBody, rigidBody.getMaxLinearVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxAngularVelocity, rigidBody, rigidBody.getMaxAngularVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigidBodyFlags, rigidBody, rigidBody.getRigidBodyFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, minCCDAdvanceCoefficient, rigidBody, rigidBody.getMinCCDAdvanceCoefficient());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxDepenetrationVelocity, rigidBody, rigidBody.getMaxDepenetrationVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, maxContactImpulse, rigidBody, rigidBody.getMaxContactImpulse());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, contactSlopCoefficient, rigidBody, rigidBody.getContactSlopCoefficient());
}

void streamRigidDynamicAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxRigidDynamic& rd)
{
	streamRigidBodyAttributes(pvdWriter, pvdRegData, rd);

	if (rd.getScene())
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, isSleeping, rd, rd.isSleeping());
	}
	else
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, isSleeping, rd, true);
	}

	// Getters don't issue warnings, just return values
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, sleepThreshold, rd, rd.getSleepThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, stabilizationThreshold, rd, rd.getStabilizationThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, rigidDynamicLockFlags, rd, rd.getRigidDynamicLockFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, wakeCounter, rd, rd.getWakeCounter());

	PxU32 positionIters, velocityIters; rd.getSolverIterationCounts(positionIters, velocityIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, positionIterations, rd, positionIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, velocityIterations, rd, velocityIters);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, contactReportThreshold, rd, rd.getContactReportThreshold());
}

void streamRigidDynamic(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxRigidDynamic& rd)
{
	const PxActor& a = rd;
	PX_ASSERT(&a == &rd);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, rd);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eRIGID_DYNAMIC);

	streamRigidDynamicAttributes(pvdWriter, pvdRegData, rd);
}

void streamRigidStatic(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxRigidStatic& rs)
{
	const PxActor& a = rs;
	PX_ASSERT(&a == &rs);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidStatic, rs);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eRIGID_STATIC);

	streamRigidActorAttributes(pvdWriter, pvdRegData, rs);
}

#if PX_SUPPORT_GPU_PHYSX

void streamPBDParticleSystemAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxPBDParticleSystem& ps)
{
	streamActorAttributes(pvdWriter, pvdRegData, ps, false);
	const NpPBDParticleSystem& npPs = static_cast<const NpPBDParticleSystem&>(ps);

	PxU32 positionIters, velocityIters; ps.getSolverIterationCounts(positionIters, velocityIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, positionIterations, ps, positionIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, velocityIterations, ps, velocityIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, simulationFilterData, ps, ps.getSimulationFilterData());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleFlags, ps, ps.getParticleFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, maxDepenetrationVelocity, ps, ps.getMaxDepenetrationVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, maxVelocity, ps, ps.getMaxVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, restOffset, ps, ps.getRestOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, contactOffset, ps, ps.getContactOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleContactOffset, ps, ps.getParticleContactOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, solidRestOffset, ps, ps.getSolidRestOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleLockFlags, ps, ps.getParticleLockFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, fluidRestOffset, ps, ps.getFluidRestOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, wind, ps, ps.getWind());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, fluidBoundaryDensityScale, ps, ps.getFluidBoundaryDensityScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, gridSizeX, ps, npPs.getCore().getGridSizeX());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, gridSizeY, ps, npPs.getCore().getGridSizeY());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, gridSizeZ, ps, npPs.getCore().getGridSizeZ());

	const PxArray<NpParticleBuffer*>& particleBuffers = npPs.mParticleBuffers;
	for (PxParticleBuffer* pb : particleBuffers)
	{
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleBuffers, ps, *pb);
	}

	const PxArray<NpParticleAndDiffuseBuffer*>& particleDiffuseBuffers = npPs.mParticleDiffuseBuffers;
	for (PxParticleBuffer* pb : particleDiffuseBuffers)
	{
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, particleBuffers, ps, *pb);
	}
}

void streamPBDParticleSystem(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxPBDParticleSystem& ps)
{
	const PxActor& a = ps;
	PX_ASSERT(&a == &ps);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDParticleSystem, ps);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::ePBD_PARTICLESYSTEM);

	streamPBDParticleSystemAttributes(pvdWriter, pvdRegData, ps);
}

void streamParticleBufferAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleBuffer& pb)
{
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, maxParticles, pb, pb.getMaxParticles());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, flatListStartIndex, pb, pb.getFlatListStartIndex());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, uniqueId, pb, pb.getUniqueId());
	streamParticleBufferName(pvdWriter, pvdRegData, pb, pb.getName());
}

void streamParticleBuffer(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleBuffer& pb)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, pb);
	streamParticleBufferAttributes(pvdWriter, pvdRegData, pb);
}

void streamDiffuseParticleParamsAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDiffuseParticleParams& diffuseParams)
{
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, threshold, diffuseParams, diffuseParams.threshold);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, lifetime, diffuseParams, diffuseParams.lifetime);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, airDrag, diffuseParams, diffuseParams.airDrag);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, bubbleDrag, diffuseParams, diffuseParams.bubbleDrag);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, buoyancy, diffuseParams, diffuseParams.buoyancy);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, kineticEnergyWeight, diffuseParams, diffuseParams.kineticEnergyWeight);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, pressureWeight, diffuseParams, diffuseParams.pressureWeight);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, divergenceWeight, diffuseParams, diffuseParams.divergenceWeight);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, collisionDecay, diffuseParams, diffuseParams.collisionDecay);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, useAccurateVelocity, diffuseParams, diffuseParams.useAccurateVelocity);
}

// Public (live-path) entry point: opens a single write scope and forwards to the explicit variant.
void streamDiffuseParticleParamsAttributes(const physx::PxDiffuseParticleParams& diffuseParams)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamDiffuseParticleParamsAttributes(pvdWriter, pvdRegData, diffuseParams);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamParticleAndDiffuseBufferAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleAndDiffuseBuffer& pb)
{
	streamParticleBufferAttributes(pvdWriter, pvdRegData, pb);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, maxDiffuseParticles, pb, pb.getMaxDiffuseParticles());
	const PxDiffuseParticleParams& diffuseParams = static_cast<const NpParticleAndDiffuseBuffer&>(pb).getDiffuseParticleParamsRef();
	streamDiffuseParticleParamsAttributes(pvdWriter, pvdRegData, diffuseParams);
}

void streamParticleAndDiffuseBuffer(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxParticleAndDiffuseBuffer& pb)
{
	{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, pb);
		streamParticleAndDiffuseBufferAttributes(pvdWriter, pvdRegData, pb);
	}

	//add PxDiffuseParticleParams
	{
		const PxDiffuseParticleParams& diffuseParams = static_cast<const NpParticleAndDiffuseBuffer&>(pb).getDiffuseParticleParamsRef();
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, diffuseParams)
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, diffuseParticleParams, pb, &diffuseParams);
	}
}

void streamDeformableBodyAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableBody& db)
{
	streamActorAttributes(pvdWriter, pvdRegData, db, false);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, deformableBodyFlags, db, db.getDeformableBodyFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, linearDamping, db, db.getLinearDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, maxLinearVelocity, db, db.getMaxLinearVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, maxDepenetrationVelocity, db, db.getMaxDepenetrationVelocity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, selfCollisionFilterDistance, db, db.getSelfCollisionFilterDistance());
	PxU32 positionIters, velocityIters; db.getSolverIterationCounts(positionIters, velocityIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, solverIterationCount, db, positionIters);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, sleepThreshold, db, db.getSleepThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, settlingThreshold, db, db.getSettlingThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, settlingDamping, db, db.getSettlingDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, wakeCounter, db, db.getWakeCounter());
	if (db.getScene())
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, isSleeping, db, db.isSleeping());
	}
	else
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, isSleeping, db, true);
	}
}

void streamDeformableVolumeAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableVolume& dv)
{
	streamDeformableBodyAttributes(pvdWriter, pvdRegData, dv);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, selfCollisionStressTolerance, dv, dv.getSelfCollisionStressTolerance());

	// Collision shape is added in NpDeformableVolume::attachShape.
	// Simulation mesh shape is added here (deferred) because the tet mesh object
	// may not exist in the OVD stream yet when attachSimulationMesh fires.
	const PxShape* shape = const_cast<PxDeformableVolume&>(dv).getShape();
	if (shape)
	{
		const PxTetrahedronMesh* simMesh = dv.getSimulationMesh();
		if (simMesh)
		{
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, simulationMesh, dv, simMesh);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, nbSimulationMeshVertices, dv, simMesh->getNbVertices());

			// Stream the simulation tet mesh once. It doesn't go through onObjectAdd
			// since it's created internally as part of PxDeformableVolumeMesh.
			if (samplerInternals->addSharedMeshIfNotSeen(simMesh, OmniPvdSharedMeshEnum::eOmniPvdTetraMesh))
			{
				streamTetMesh(pvdWriter, pvdRegData, *simMesh);
				const PxPhysics& physics = static_cast<PxPhysics&>(NpPhysics::getInstance());
				OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tetrahedronMeshes, physics, *simMesh);
				OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, simulationMeshShapes, dv, *simMesh);
			}
		}

		const PxTetrahedronMesh* collMesh = dv.getCollisionMesh();
		if (collMesh)
		{
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, collisionMesh, dv, collMesh);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, nbCollisionMeshVertices, dv, collMesh->getNbVertices());
		}
	}
}

// Public (live-path) entry point: opens a single write scope and forwards to the explicit variant.
void streamDeformableVolumeAttributes(const physx::PxDeformableVolume& dv)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamDeformableVolumeAttributes(pvdWriter, pvdRegData, dv);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamDeformableVolume(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableVolume& dv)
{
	const PxActor& a = dv;
	PX_ASSERT(&a == &dv);

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolume, dv);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eDEFORMABLE_VOLUME);

	streamDeformableVolumeAttributes(pvdWriter, pvdRegData, dv);
}

void streamDeformableSurfaceAttributes(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableSurface& ds)
{
	streamDeformableBodyAttributes(pvdWriter, pvdRegData, ds);

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, nbCollisionPairUpdatesPerTimestep, ds, ds.getNbCollisionPairUpdatesPerTimestep());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, nbCollisionSubsteps, ds, ds.getNbCollisionSubsteps());

	// Shape is now added in NpDeformableSurface::attachShape.
	// Here we only stream vertex count for reference.
	const PxShape* shape = const_cast<PxDeformableSurface&>(ds).getShape();
	if (shape)
	{
		const PxGeometry& geom = shape->getGeometry();
		if (geom.getType() == PxGeometryType::eTRIANGLEMESH)
		{
			const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
			if (triGeom.triangleMesh)
			{
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, nbVertices, ds, triGeom.triangleMesh->getNbVertices());
			}
		}
	}
}

// Public (live-path) entry point: opens a single write scope and forwards to the explicit variant.
void streamDeformableSurfaceAttributes(const physx::PxDeformableSurface& ds)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamDeformableSurfaceAttributes(pvdWriter, pvdRegData, ds);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamDeformableSurface(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableSurface& ds)
{
	const PxActor& a = ds;
	PX_ASSERT(&a == &ds);

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurface, ds);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eDEFORMABLE_SURFACE);

	streamDeformableSurfaceAttributes(pvdWriter, pvdRegData, ds);
}

#endif

void streamArticulationJoint(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationJointReducedCoordinate& jointRef)
{
	const PxU32 degreesOfFreedom = PxArticulationAxis::eCOUNT;

	// make sure size matches the size used in the PVD description
	PX_ASSERT(sizeof(PxArticulationMotion::Enum) == getOmniPvdDataTypeSize<OmniPvdDataType::eUINT32>());
	PX_ASSERT(sizeof(PxArticulationDriveType::Enum) == getOmniPvdDataTypeSize<OmniPvdDataType::eUINT32>());

	PxArticulationJointType::Enum jointType = jointRef.getJointType();
	const PxArticulationLink* parentPxLinkPtr = &jointRef.getParentArticulationLink();
	const PxArticulationLink* childPxLinkPtr = &jointRef.getChildArticulationLink();
	PxArticulationMotion::Enum motions[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		motions[ax] = jointRef.getMotion(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal armatures[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		armatures[ax] = jointRef.getArmature(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal coefficient = jointRef.getFrictionCoefficient();
	PxReal maxJointV = jointRef.getMaxJointVelocity();
	PxReal positions[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		positions[ax] = jointRef.getJointPosition(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal velocitys[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		velocitys[ax] = jointRef.getJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
	const char* concreteTypeName = jointRef.getConcreteTypeName();
	PxU32 concreteTypeNameLen = PxU32(strnlen(concreteTypeName, OMNI_PVD_MAX_STRING_LENGTH)) + 1;
	PxReal lowlimits[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		lowlimits[ax] = jointRef.getLimitParams(static_cast<PxArticulationAxis::Enum>(ax)).low;
	PxReal highlimits[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		highlimits[ax] = jointRef.getLimitParams(static_cast<PxArticulationAxis::Enum>(ax)).high;
	PxReal stiffnesss[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		stiffnesss[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).stiffness;
	PxReal dampings[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		dampings[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).damping;
	PxReal maxforces[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		maxforces[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).maxForce;
	PxReal maxefforts[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		maxefforts[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).envelope.maxEffort;
	PxReal maxactuatorvelocities[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		maxactuatorvelocities[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).envelope.maxActuatorVelocity;
	PxReal velocitydependentresistances[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		velocitydependentresistances[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).envelope.velocityDependentResistance;
	PxReal speedeffortgradients[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		speedeffortgradients[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).envelope.speedEffortGradient;
	PxArticulationDriveType::Enum drivetypes[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		drivetypes[ax] = jointRef.getDriveParams(static_cast<PxArticulationAxis::Enum>(ax)).driveType;
	PxReal drivetargets[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		drivetargets[ax] = jointRef.getDriveTarget(static_cast<PxArticulationAxis::Enum>(ax));
	PxReal drivevelocitys[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		drivevelocitys[ax] = jointRef.getDriveVelocity(static_cast<PxArticulationAxis::Enum>(ax));

	PxReal staticfrictionefforts[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		staticfrictionefforts[ax] = jointRef.getFrictionParams(static_cast<PxArticulationAxis::Enum>(ax)).staticFrictionEffort;
	PxReal dynamicfrictionefforts[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		dynamicfrictionefforts[ax] = jointRef.getFrictionParams(static_cast<PxArticulationAxis::Enum>(ax)).dynamicFrictionEffort;
	PxReal viscousFrictionCoefficients[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		viscousFrictionCoefficients[ax] = jointRef.getFrictionParams(static_cast<PxArticulationAxis::Enum>(ax)).viscousFrictionCoefficient;
	PxReal maxJointDofV[degreesOfFreedom];
	for (PxU32 ax = 0; ax < degreesOfFreedom; ++ax)
		maxJointDofV[ax] = jointRef.getMaxJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
	// Current parent/child frames: normally streamed via the setParentPose/setChildPose
	// hooks, but emitted here too so the create-time stream (and a full-state snapshot)
	// carries the current joint frames without depending on a later setter call.
	const physx::PxTransform jointParentPose = jointRef.getParentPose();
	const physx::PxTransform jointChildPose = jointRef.getChildPose();

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointRef);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, type, jointRef, jointType);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, parentLink, jointRef, parentPxLinkPtr);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, childLink, jointRef, childPxLinkPtr);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, parentTranslation, jointRef, jointParentPose.p);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, parentRotation, jointRef, jointParentPose.q);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, childTranslation, jointRef, jointChildPose.p);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, childRotation, jointRef, jointChildPose.q);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, motion, jointRef, motions, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, armature, jointRef, armatures, degreesOfFreedom);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, frictionCoefficient, jointRef, coefficient);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, staticFrictionEffort,
								jointRef, staticfrictionefforts, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, dynamicFrictionEffort, jointRef, dynamicfrictionefforts, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate,
								viscousFrictionCoefficient, jointRef, viscousFrictionCoefficients, degreesOfFreedom);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, maxJointVelocity, jointRef, maxJointV);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate,
								maxJointDofVelocity, jointRef, maxJointDofV, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointPosition, jointRef, positions, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointVelocity, jointRef, velocitys, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, concreteTypeName, jointRef, concreteTypeName, concreteTypeNameLen);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, limitLow, jointRef, lowlimits, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, limitHigh, jointRef, highlimits, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveStiffness, jointRef, stiffnesss, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveDamping, jointRef, dampings, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveMaxForce, jointRef, maxforces, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveMaxEffort, jointRef, maxefforts, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveMaxActuatorVelocity, jointRef, maxactuatorvelocities, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveVelocityDependentResistance, jointRef, velocitydependentresistances, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveSpeedEffortGradient, jointRef, speedeffortgradients, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveType, jointRef, drivetypes, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveTarget, jointRef, drivetargets, degreesOfFreedom);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveVelocity, jointRef, drivevelocitys, degreesOfFreedom);
}

void streamArticulationLink(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationLink& al)
{
	const PxActor& a = al;
	PX_ASSERT(&a == &al);  // if this changes, we would have to cast in a way such that the addresses are the same

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, al);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, type, a, PxActorType::eARTICULATION_LINK);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, articulation, al, &al.getArticulation());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, CFMScale, al, al.getCfmScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, inboundJointDOF, al, al.getInboundJointDof());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationLink, inboundJoint, al, al.getInboundJoint());

	streamRigidBodyAttributes(pvdWriter, pvdRegData, al);
}

void streamArticulationMimicJoint(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationMimicJoint& mj)
{
	const NpArticulationMimicJoint& np = static_cast<const NpArticulationMimicJoint&>(mj);

	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, mj);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, articulation, mj, &mj.getArticulation());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, jointA, mj, &np.getJointA());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, jointB, mj, &np.getJointB());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, axisA, mj, np.getAxisA());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, axisB, mj, np.getAxisB());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, gearRatio, mj, mj.getGearRatio());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, offset, mj, mj.getOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, naturalFrequency, mj, mj.getNaturalFrequency());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, dampingRatio, mj, mj.getDampingRatio());
}

// Public (live-path) entry point: opens a single write scope and forwards to the explicit variant.
void streamArticulationMimicJoint(const physx::PxArticulationMimicJoint& mj)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamArticulationMimicJoint(pvdWriter, pvdRegData, mj);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamArticulation(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxArticulationReducedCoordinate& art)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, art);
	PxU32 solverIterations[2]; art.getSolverIterationCounts(solverIterations[0], solverIterations[1]);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, positionIterations, art, solverIterations[0]);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, velocityIterations, art, solverIterations[1]);

	const NpScene* npScene = static_cast<const NpScene*>(art.getScene());

	if (npScene)
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, isSleeping, art, art.isSleeping());
	}
	else
	{
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, isSleeping, art, false);
	}

	// Getters don't issue warnings, just return values
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, sleepThreshold, art, art.getSleepThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, stabilizationThreshold, art, art.getStabilizationThreshold());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, wakeCounter, art, art.getWakeCounter());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, worldBounds, art, art.getWorldBounds());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, articulationFlags, art, art.getArticulationFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, dofs, art, art.getDofs());
}

void streamAggregate(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxAggregate& agg)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, agg);
	PxU32 actorCount = agg.getNbActors();
	for (PxU32 i = 0; i < actorCount; ++i)
	{
		PxActor* a; agg.getActors(&a, 1, i);
		OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, actors, agg, *a);
	}
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, selfCollision, agg, agg.getSelfCollision());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, environmentID, agg, agg.getEnvironmentID());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, maxNbShapes, agg, agg.getMaxNbShapes());
	PxScene* scene = static_cast<const NpAggregate&>(agg).getNpScene();  // because PxAggregate::getScene() is not marked const
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxAggregate, scene, agg, scene);
}

void streamPBDMaterial(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxPBDMaterial& m)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, m);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, friction, m, m.getFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, damping, m, m.getDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, adhesion, m, m.getAdhesion());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, gravityScale, m, m.getGravityScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, adhesionRadiusScale, m, m.getAdhesionRadiusScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, viscosity, m, m.getViscosity());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, vorticityConfinement, m, m.getVorticityConfinement());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, surfaceTension, m, m.getSurfaceTension());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, cohesion, m, m.getCohesion());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, lift, m, m.getLift());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, drag, m, m.getDrag());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, CFLCoefficient, m, m.getCFLCoefficient());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, particleFrictionScale, m, m.getParticleFrictionScale());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, particleAdhesionScale, m, m.getParticleAdhesionScale());
}

void streamFEMClothMaterial(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableSurfaceMaterial& m)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, m);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, youngsModulus, m, m.getYoungsModulus());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, poissons, m, m.getPoissons());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, dynamicFriction, m, m.getDynamicFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, elasticityDamping, m, m.getElasticityDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, thickness, m, m.getThickness());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, bendingStiffness, m, m.getBendingStiffness());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, bendingDamping, m, m.getBendingDamping());
}

void streamFEMSoBoMaterial(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableVolumeMaterial& m)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, m);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, youngsModulus, m, m.getYoungsModulus());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, poissons, m, m.getPoissons());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, dynamicFriction, m, m.getDynamicFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, elasticityDamping, m, m.getElasticityDamping());
}

void streamMaterial(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxMaterial& m)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, m);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, flags, m, m.getFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, frictionCombineMode, m, m.getFrictionCombineMode());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, restitutionCombineMode, m, m.getRestitutionCombineMode());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, staticFriction, m, m.getStaticFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, dynamicFriction, m, m.getDynamicFriction());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, restitution, m, m.getRestitution());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, damping, m, m.getDamping());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxMaterial, dampingCombineMode, m, m.getDampingCombineMode());
}

// Explicit form (the caller already holds a write scope), used by the full-state snapshot's streamShape.
void streamShapeMaterials(OmniPvdWriter* pvdWriter, const physx::OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxShape& shape, physx::PxMaterial* const* mats, physx::PxU32 nbrMaterials)
{
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, materials, shape, mats, nbrMaterials);
}

// Scope-opening form, used live by NpShape::setMaterialsInternal (template instantiation for PxMaterial).
void streamShapeMaterials(const physx::PxShape& shape, physx::PxMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	streamShapeMaterials(pvdWriter, pvdRegData, shape, mats, nbrMaterials);
	OMNI_PVD_WRITE_SCOPE_END
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxDeformableSurfaceMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxDeformableVolumeMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}

void streamShapeMaterials(const physx::PxShape& shape, physx::PxPBDMaterial* const * mats, physx::PxU32 nbrMaterials)
{
	PX_UNUSED(shape);
	PX_UNUSED(mats);
	PX_UNUSED(nbrMaterials);
}


void streamShape(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxShape& shape)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, shape);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, isExclusive, shape, shape.isExclusive());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, geom, shape, &shape.getGeometry());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, contactOffset, shape, shape.getContactOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, restOffset, shape, shape.getRestOffset());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, densityForFluid, shape, shape.getDensityForFluid());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, torsionalPatchRadius, shape, shape.getTorsionalPatchRadius());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, minTorsionalPatchRadius, shape, shape.getMinTorsionalPatchRadius());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, shapeFlags, shape, shape.getFlags());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, simulationFilterData, shape, shape.getSimulationFilterData());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, queryFilterData, shape, shape.getQueryFilterData());

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxShape, localPose, shape, shape.getLocalPose());


	// Stream shape materials. Try deformable-specific getters first to get the
	// correct pointer values (multiple inheritance can adjust PxMaterial* pointers
	// away from the original PxDeformable*Material* address used in OVD registration).
	const int nbrMaterials = shape.getNbMaterials();
	void* tmpMaterials = PX_ALLOC(sizeof(void*) * nbrMaterials, "tmpMaterials");
	// Use internal shape core flags to determine the actual material type.
	// All public getMaterials overloads return materials regardless of type,
	// but pointer values differ due to multiple inheritance - we must use
	// the getter matching the registered OVD material type.
	const NpShape& npShape = static_cast<const NpShape&>(shape);
	const PxShapeCoreFlags coreFlags = npShape.getCore().mShapeCoreFlags;
	if (coreFlags & PxShapeCoreFlag::eDEFORMABLE_VOLUME_SHAPE)
	{
		physx::PxU32 nbrMats = shape.getDeformableVolumeMaterials(reinterpret_cast<PxDeformableVolumeMaterial**>(tmpMaterials), nbrMaterials);
		if (nbrMats > 0)
			streamShapeMaterials(pvdWriter, pvdRegData, shape, reinterpret_cast<PxMaterial* const*>(tmpMaterials), nbrMats);
	}
	else if (coreFlags & PxShapeCoreFlag::eDEFORMABLE_SURFACE_SHAPE)
	{
		physx::PxU32 nbrMats = shape.getDeformableSurfaceMaterials(reinterpret_cast<PxDeformableSurfaceMaterial**>(tmpMaterials), nbrMaterials);
		if (nbrMats > 0)
			streamShapeMaterials(pvdWriter, pvdRegData, shape, reinterpret_cast<PxMaterial* const*>(tmpMaterials), nbrMats);
	}
	else
	{
		physx::PxU32 nbrMats = shape.getMaterials(reinterpret_cast<PxMaterial**>(tmpMaterials), nbrMaterials);
		if (nbrMats > 0)
			streamShapeMaterials(pvdWriter, pvdRegData, shape, reinterpret_cast<PxMaterial* const*>(tmpMaterials), nbrMats);
	}

	PX_FREE(tmpMaterials);
}

void streamBVH(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxBVH& bvh)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxBVH, bvh);
}

void streamDeVoMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxDeformableVolumeMesh& mesh)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMesh, mesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMesh, collisionMesh, mesh, mesh.getCollisionMesh());
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMesh, simulationMesh, mesh, mesh.getSimulationMesh());
}

void streamTetMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxTetrahedronMesh& mesh)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, mesh);
	//this gets done at the bottom now
	const PxU32 tetrahedronCount = mesh.getNbTetrahedrons();
	const PxU32 has16BitIndices = mesh.getTetrahedronMeshFlags() & physx::PxTetrahedronMeshFlag::e16_BIT_INDICES;
	const void* indexBuffer = mesh.getTetrahedrons();
	const PxVec3* vertexBuffer = mesh.getVertices();
	const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
	const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
	//TODO: not needed to copy this
	const PxU32 nbrVerts = mesh.getNbVertices();
	const PxU32 nbrTets = mesh.getNbTetrahedrons();
	float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
	PxU32 vertIndex = 0;
	for (PxU32 v = 0; v < nbrVerts; v++)
	{
		tmpVerts[vertIndex + 0] = vertexBuffer[v].x;
		tmpVerts[vertIndex + 1] = vertexBuffer[v].y;
		tmpVerts[vertIndex + 2] = vertexBuffer[v].z;
		vertIndex += 3;
	}
	PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(nbrTets * 4), "tmpIndices");
	const PxU32 totalIndexCount = tetrahedronCount * 4;
	if (has16BitIndices)
	{
		for (PxU32 i = 0; i < totalIndexCount; ++i)
		{
			tmpIndices[i] = shortIndices[i];
		}
	}
	else
	{
		for (PxU32 i = 0; i < totalIndexCount; ++i)
		{
			tmpIndices[i] = intIndices[i];
		}
	}
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, verts, mesh, tmpVerts, 3 * nbrVerts);
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, tets, mesh, tmpIndices, 4 * nbrTets);
	PX_FREE(tmpVerts);
	PX_FREE(tmpIndices);
}

void streamTriMesh(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxTriangleMesh& mesh)
{
	if (samplerInternals->addSharedMeshIfNotSeen(&mesh, OmniPvdSharedMeshEnum::eOmniPvdTriMesh))
	{
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, mesh);
		//this gets done at the bottom now
		const PxU32 triangleCount = mesh.getNbTriangles();
		const PxU32 has16BitIndices = mesh.getTriangleMeshFlags() & physx::PxTriangleMeshFlag::e16_BIT_INDICES;
		const void* indexBuffer = mesh.getTriangles();
		const PxVec3* vertexBuffer = mesh.getVertices();
		const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
		const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
		//TODO: not needed to copy this
		const PxU32 nbrVerts = mesh.getNbVertices();
		const PxU32 nbrTris = mesh.getNbTriangles();
		float* tmpVerts = (float*)PX_ALLOC(sizeof(float)*(nbrVerts * 3), "tmpVerts");
		PxU32 vertIndex = 0;
		for (PxU32 v = 0; v < nbrVerts; v++)
		{
			tmpVerts[vertIndex + 0] = vertexBuffer[v].x;
			tmpVerts[vertIndex + 1] = vertexBuffer[v].y;
			tmpVerts[vertIndex + 2] = vertexBuffer[v].z;
			vertIndex += 3;
		}
		PxU32* tmpIndices = (PxU32*)PX_ALLOC(sizeof(PxU32)*(nbrTris * 3), "tmpIndices");
		const PxU32 totalIndexCount = triangleCount * 3;
		if (has16BitIndices)
		{
			for (PxU32 i = 0; i < totalIndexCount; ++i)
			{
				tmpIndices[i] = shortIndices[i];
			}
		}
		else
		{
			for (PxU32 i = 0; i < totalIndexCount; ++i)
			{
				tmpIndices[i] = intIndices[i];
			}
		}
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, verts, mesh, tmpVerts, 3 * nbrVerts);
		OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, tris, mesh, tmpIndices, 3 * nbrTris);
		PX_FREE(tmpVerts);
		PX_FREE(tmpIndices);
	}
}

void streamTriMeshGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxTriangleMeshGeometry& g)
{
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, g);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, scale, g, g.scale.scale);
	streamTriMesh(pvdWriter, pvdRegData, *g.triangleMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, triangleMesh, g, g.triangleMesh);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, meshFlags, g, g.meshFlags);
}

void OmniPvdPxSampler::streamSceneContacts(physx::NpScene& scene)
{
	if (!isSampling()) return;
	PxsContactManagerOutputIterator outputIter;
	Sc::ContactIterator contactIter;
	scene.getScScene().initContactsIterator(contactIter, outputIter);
	Sc::ContactIterator::Pair* pair;
	PxU32 pairCount = 0;
	PxArray<PxActor*> pairsActors;
	PxArray<PxU32> pairsContactCounts;
	PxArray<PxVec3> pairsContactPoints;
	PxArray<PxVec3> pairsContactNormals;
	PxArray<PxReal> pairsContactSeparations;
	PxArray<PxShape*> pairsContactShapes;
	PxArray<PxU32> pairsContactFacesIndices;
	PxArray<PxReal> pairsContactImpulses;
	PxArray<PxU32> pairsFrictionAnchorCounts;
	PxArray<PxVec3> pairsFrictionAnchorPositions;
	PxArray<PxVec3> pairsFrictionAnchorNormals;
	PxArray<PxVec3> pairsFrictionAnchorImpulses;

	while ((pair = contactIter.getNextPair()) != NULL)
	{
		PxU32 pairContactCount = 0;
		PxU32 pairFrictionAnchorCount = 0;
		Sc::Contact* contact = NULL;
		Sc::FrictionAnchor* anchor = NULL;
		bool firstContact = true;
		while ((contact = pair->getNextContact()) != NULL)
		{
			if (firstContact) {
				pairsActors.pushBack(pair->getActor0());
				pairsActors.pushBack(pair->getActor1());
				++pairCount;
				firstContact = false;
			}
			++pairContactCount;
			pairsContactPoints.pushBack(contact->point);
			pairsContactNormals.pushBack(contact->normal);
			pairsContactSeparations.pushBack(contact->separation);
			pairsContactShapes.pushBack(contact->shape0);
			pairsContactShapes.pushBack(contact->shape1);
			pairsContactFacesIndices.pushBack(contact->faceIndex0);
			pairsContactFacesIndices.pushBack(contact->faceIndex1);
			pairsContactImpulses.pushBack(contact->normalForce);
		}
		if (pairContactCount)
		{
			pairsContactCounts.pushBack(pairContactCount);
		}
		while ((anchor = pair->getNextFrictionAnchor()) != NULL)
		{
			++pairFrictionAnchorCount;
			pairsFrictionAnchorPositions.pushBack(anchor->point);
			pairsFrictionAnchorNormals.pushBack(anchor->normal);
			pairsFrictionAnchorImpulses.pushBack(anchor->impulse);
		}
		if (pairFrictionAnchorCount)
		{
			pairsFrictionAnchorCounts.pushBack(pairFrictionAnchorCount);
		}

	}

	if (pairCount == 0) return;

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairCount, scene, pairCount);
	const PxU32 actorCount = pairsActors.size();
	const PxActor** actors = actorCount ? const_cast<const PxActor**>(pairsActors.begin()) : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsActors, scene, actors, actorCount);
	PxU32 nbContactCount = pairsContactCounts.size();
	PxU32* contactCounts = nbContactCount ? pairsContactCounts.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactCounts, scene, contactCounts, nbContactCount);
	PxU32 contactPointFloatCount = pairsContactPoints.size() * 3;
	PxReal* contactPoints = contactPointFloatCount ? &pairsContactPoints[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactPoints, scene, contactPoints, contactPointFloatCount);
	PxU32 contactNormalFloatCount = pairsContactNormals.size() * 3;
	PxReal* contactNormals = contactNormalFloatCount ? &pairsContactNormals[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactNormals, scene, contactNormals, contactNormalFloatCount);
	PxU32 contactSeparationCount = pairsContactSeparations.size();
	PxReal* contactSeparations = contactSeparationCount ? pairsContactSeparations.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactSeparations, scene, contactSeparations, contactSeparationCount);
	PxU32 contactShapeCount = pairsContactShapes.size();
	const PxShape** contactShapes = contactShapeCount ? const_cast<const PxShape**>(pairsContactShapes.begin()) : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactShapes, scene, contactShapes, contactShapeCount);
	PxU32 contactFacesIndexCount = pairsContactFacesIndices.size();
	PxU32* contactFacesIndices = contactFacesIndexCount ? pairsContactFacesIndices.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactFacesIndices, scene, contactFacesIndices, contactFacesIndexCount);
	PxU32 contactImpulseCount = pairsContactImpulses.size();
	PxReal* contactImpulses = contactImpulseCount ? pairsContactImpulses.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsContactImpulses, scene, contactImpulses, contactImpulseCount);
	PxU32 nbFrictionAnchorCount = pairsFrictionAnchorCounts.size();
	PxU32* frictionAnchorCounts = nbFrictionAnchorCount ? pairsFrictionAnchorCounts.begin() : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsFrictionAnchorCounts, scene, frictionAnchorCounts, nbFrictionAnchorCount);
	PxU32 frictionAnchorPositionFloatCount = pairsFrictionAnchorPositions.size() * 3;
	PxReal* frictionAnchorPositions = frictionAnchorPositionFloatCount ? &pairsFrictionAnchorPositions[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsFrictionAnchorPositions, scene, frictionAnchorPositions, frictionAnchorPositionFloatCount);
	PxU32 frictionAnchorNormalFloatCount = pairsFrictionAnchorNormals.size() * 3;
	PxReal* frictionAnchorNormals = frictionAnchorNormalFloatCount ? &pairsFrictionAnchorNormals[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsFrictionAnchorNormals, scene, frictionAnchorNormals, frictionAnchorNormalFloatCount);
	PxU32 frictionAnchorImpulseFloatCount = pairsFrictionAnchorImpulses.size() * 3;
	PxReal* frictionAnchorImpulses = frictionAnchorImpulseFloatCount ? &pairsFrictionAnchorImpulses[0].x : NULL;
	OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxScene, pairsFrictionAnchorImpulses, scene, frictionAnchorImpulses, frictionAnchorImpulseFloatCount);

	OMNI_PVD_WRITE_SCOPE_END

}

OmniPvdPxSampler::OmniPvdPxSampler()
{
	samplerInternals = PX_NEW(OmniPvdSamplerInternals)();

	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
	samplerInternals->mIsSampling = false;
}

OmniPvdPxSampler::~OmniPvdPxSampler()
{
	PX_DELETE(samplerInternals);
}

bool OmniPvdPxSampler::stopSampling()
{
	if (!samplerInternals) return false;
	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
	samplerInternals->mIsSampling = false;
	return true;
}

bool OmniPvdPxSampler::isSampling()
{
	if (!samplerInternals) return false;
	physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
	return samplerInternals->mIsSampling;
}

// Number of objects fetched per batch by the snapshot world walk. The object lists are read into
// a fixed-size stack buffer in batches so the one-time snapshot does not heap-allocate per list.
static const physx::PxU32 gOvdSnapshotBatch = 64;

// Enumerate a PxPhysics-level resource list and (re-)emit each element via onObjectAdd.
// physics is a name in scope at the use site. Emission uses the ambient pvdWriter / pvdRegData
// from the single write scope that snapshotAll() holds around all object creation (so the whole
// snapshot is one exclusive-writer lock acquisition), via the explicit onObjectAdd variant.
#define OVD_SNAPSHOT_PHYSICS_LIST(PxType, GetNbFn, GetFn)                                   \
	{                                                                                       \
		const physx::PxU32 _nb = physics.GetNbFn();                                         \
		PxType* _batch[gOvdSnapshotBatch];                                                  \
		for (physx::PxU32 _off = 0; _off < _nb; _off += gOvdSnapshotBatch)                  \
		{                                                                                   \
			const physx::PxU32 _got = physics.GetFn(_batch, gOvdSnapshotBatch, _off);       \
			for (physx::PxU32 _i = 0; _i < _got; ++_i)                                      \
				onObjectAdd(pvdWriter, pvdRegData, *_batch[_i]);                            \
		}                                                                                   \
	}

bool OmniPvdPxSampler::snapshotAll()
{
	if (!samplerInternals) return false;

	// Ensure sampling is on: emission flows through onObjectAdd(), which early-returns
	// when sampling is off.
	{
		physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
		samplerInternals->mIsSampling = true;
	}

	physx::NpPhysics& npPhysics = physx::NpPhysics::getInstance();
	physx::PxPhysics& physics = static_cast<physx::PxPhysics&>(npPhysics);

	// (0) Register the full class/attribute schema on the bound stream, then emit the
	//     PxOmniPvdMetaData + PxPhysics singletons. initOmniPvd() does both, in that order.
	samplerInternals->mPvdStream.initOmniPvd();

	// (0c) Clear the cross-stream shared-mesh dedup so mesh vertex/index data re-emits.
	{
		physx::PxMutex::ScopedLock geomLock(samplerInternals->mSharedGeomsMutex);
		samplerInternals->mSharedMeshesMap.clear();
	}

	// All object creation runs under ONE write scope: a single exclusive-writer lock acquisition for
	// the whole snapshot (rather than one per batch), so the created world is emitted atomically. The
	// stream* helpers and the onObjectAdd batch overload use the _EXPLICIT commands and take this
	// pvdWriter / pvdRegData, so nothing re-opens a nested scope. The per-scene pass (4) stays OUTSIDE
	// this scope: emitSceneStateToOmniPvd opens its own, and nesting it here would relock the writer.
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

	// (1) Shared leaf resources first (materials + meshes), since shapes reference them.
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxMaterial,         getNbMaterials,         getMaterials)
#if PX_SUPPORT_GPU_PHYSX
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxDeformableSurfaceMaterial, getNbDeformableSurfaceMaterials, getDeformableSurfaceMaterials)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxDeformableVolumeMaterial,  getNbDeformableVolumeMaterials,  getDeformableVolumeMaterials)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxPBDMaterial,      getNbPBDMaterials,      getPBDMaterials)
#endif
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxConvexMesh,       getNbConvexMeshes,      getConvexMeshes)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxTriangleMesh,     getNbTriangleMeshes,    getTriangleMeshes)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxHeightField,      getNbHeightFields,      getHeightFields)
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxTetrahedronMesh,  getNbTetrahedronMeshes, getTetrahedronMeshes)
	// Deformable volume meshes: the cooked wrapper bundling the collision + simulation tetrahedron
	// meshes emitted just above. There is no PxPhysics-level getter, so enumerate them from the
	// factory (which tracks them); emit after the tetrahedron meshes so the wrapper's mesh
	// references resolve. Deformable volumes are GPU-only, so the factory getters live behind
	// PX_SUPPORT_GPU_PHYSX.
#if PX_SUPPORT_GPU_PHYSX
	{
		physx::NpFactory& dvmFactory = physx::NpFactory::getInstance();
		const physx::PxU32 nbDvm = dvmFactory.getNbDeformableVolumeMeshes();
		physx::PxDeformableVolumeMesh* dvmBatch[gOvdSnapshotBatch];
		for (physx::PxU32 off = 0; off < nbDvm; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = dvmFactory.getDeformableVolumeMeshes(dvmBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				onObjectAdd(pvdWriter, pvdRegData, *dvmBatch[i]);
		}
	}
#endif
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxBVH,              getNbBVHs,              getBVHs)

	// (2) Shapes (global, de-duplicated set), referenced by actors below.
	OVD_SNAPSHOT_PHYSICS_LIST(physx::PxShape,            getNbShapes,            getShapes)

	// (3) Every object the factory tracks, created once at the PxPhysics/factory level, exactly as
	//     from-start recording emits each on creation (independent of any scene). The per-scene pass
	//     below then only emits scene membership. This keeps a snapshot and a from-start recording
	//     structurally identical, and means each object (particle buffers included) is created exactly
	//     once, so no dedup is needed.
	{
		physx::NpFactory& factory = physx::NpFactory::getInstance();
#if PX_SUPPORT_GPU_PHYSX
		// Particle buffers before the particle systems, so the systems' particleBuffers edges resolve.
		const physx::PxU32 nbBufs = factory.getNbParticleBuffers();
		physx::PxParticleBuffer* bufBatch[gOvdSnapshotBatch];
		for (physx::PxU32 off = 0; off < nbBufs; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getParticleBuffers(bufBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				if (bufBatch[i]) onObjectAdd(pvdWriter, pvdRegData, *bufBatch[i]);
		}
#endif
		// Actors, enumerated per factory tracking set, exactly as from-start recording emits each:
		// onObjectAdd creates the object with its shapes / particleBuffers edges. Deformables additionally
		// need the PxDeformableBody.shapes edge re-emitted here (otherwise emitted only at attachShape; the
		// viewer reaches the collision/sim mesh through it).

		// Rigid statics, then rigid dynamics (each tracked in its own factory set).
		physx::PxRigidStatic* rigidStaticBatch[gOvdSnapshotBatch];
		const physx::PxU32 nbRigidStatics = factory.getNbRigidStatics();
		for (physx::PxU32 off = 0; off < nbRigidStatics; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getRigidStatics(rigidStaticBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				if (rigidStaticBatch[i]) onObjectAdd(pvdWriter, pvdRegData, *rigidStaticBatch[i]);
		}
		physx::PxRigidDynamic* rigidDynamicBatch[gOvdSnapshotBatch];
		const physx::PxU32 nbRigidDynamics = factory.getNbRigidDynamics();
		for (physx::PxU32 off = 0; off < nbRigidDynamics; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getRigidDynamics(rigidDynamicBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
			{
				if (!rigidDynamicBatch[i])
					continue;
				onObjectAdd(pvdWriter, pvdRegData, *rigidDynamicBatch[i]);
			}
		}
#if PX_SUPPORT_GPU_PHYSX
		physx::PxActor* actorBatch[gOvdSnapshotBatch]; // shared batch buffer for the GPU actor types below
		// Deformable surfaces, with the PxDeformableBody.shapes edge re-emitted.
		const physx::PxU32 nbDefSurf = factory.getNbDeformableSurfaces();
		for (physx::PxU32 off = 0; off < nbDefSurf; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getDeformableSurfaces(actorBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
			{
				physx::PxActor* a = actorBatch[i];
				if (!a)
					continue;
				onObjectAdd(pvdWriter, pvdRegData, *a);
				if (physx::PxShape* sh = static_cast<physx::PxDeformableSurface*>(a)->getShape())
				{
					OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, shapes, static_cast<physx::PxDeformableBody&>(*a), *sh)
				}
			}
		}
		// Deformable volumes, with the PxDeformableBody.shapes edge re-emitted.
		const physx::PxU32 nbDefVol = factory.getNbDeformableVolumes();
		for (physx::PxU32 off = 0; off < nbDefVol; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getDeformableVolumes(actorBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
			{
				physx::PxActor* a = actorBatch[i];
				if (!a)
					continue;
				onObjectAdd(pvdWriter, pvdRegData, *a);
				if (physx::PxShape* sh = static_cast<physx::PxDeformableVolume*>(a)->getShape())
				{
					OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, shapes, static_cast<physx::PxDeformableBody&>(*a), *sh)
				}
			}
		}
		// PBD particle systems.
		const physx::PxU32 nbPS = factory.getNbPBDParticleSystems();
		for (physx::PxU32 off = 0; off < nbPS; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getPBDParticleSystems(actorBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				if (actorBatch[i]) onObjectAdd(pvdWriter, pvdRegData, *actorBatch[i]);
		}
#endif
		// Articulations, with their links, then inbound joints, then mimic joints (referenced-before-referrer).
		const physx::PxU32 nbArts = factory.getNbArticulations();
		physx::PxArticulationReducedCoordinate* artBatch[gOvdSnapshotBatch];
		for (physx::PxU32 off = 0; off < nbArts; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getArticulations(artBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
			{
				physx::PxArticulationReducedCoordinate* art = artBatch[i];
				if (!art)
					continue;
				onObjectAdd(pvdWriter, pvdRegData, *art);
				streamArticulationName(pvdWriter, pvdRegData, *art, art->getName()); // re-emit the name (set live only via setName, not at create)
				const physx::PxU32 nbLinks = art->getNbLinks();
				physx::PxArticulationLink* linkBatch[gOvdSnapshotBatch];
				for (physx::PxU32 lo = 0; lo < nbLinks; lo += gOvdSnapshotBatch)
				{
					const physx::PxU32 gotL = art->getLinks(linkBatch, gOvdSnapshotBatch, lo);
					for (physx::PxU32 l = 0; l < gotL; ++l)
					{
						onObjectAdd(pvdWriter, pvdRegData, *linkBatch[l]);     // creates the link and adds it to PxArticulationReducedCoordinate.links
					}
					for (physx::PxU32 l = 0; l < gotL; ++l)
					{
						physx::PxArticulationJointReducedCoordinate* joint = linkBatch[l]->getInboundJoint();
						if (joint)
						{
							onObjectAdd(pvdWriter, pvdRegData, *joint);
							streamArticulationJointName(pvdWriter, pvdRegData, *joint, joint->getName()); // re-emit the name (set live only via setName)
						}
					}
				}
				const physx::PxU32 nbMimic = art->getNbMimicJoints();
				physx::PxArticulationMimicJoint* mimicBatch[gOvdSnapshotBatch];
				for (physx::PxU32 mo = 0; mo < nbMimic; mo += gOvdSnapshotBatch)
				{
					const physx::PxU32 gotM = art->getMimicJoints(mimicBatch, gOvdSnapshotBatch, mo);
					for (physx::PxU32 m = 0; m < gotM; ++m)
						streamArticulationMimicJoint(pvdWriter, pvdRegData, *mimicBatch[m]);
				}
			}
		}
		// Aggregates.
		const physx::PxU32 nbAggs = factory.getNbAggregates();
		physx::PxAggregate* aggBatch[gOvdSnapshotBatch];
		for (physx::PxU32 off = 0; off < nbAggs; off += gOvdSnapshotBatch)
		{
			const physx::PxU32 got = factory.getAggregates(aggBatch, gOvdSnapshotBatch, off);
			for (physx::PxU32 i = 0; i < got; ++i)
				if (aggBatch[i]) onObjectAdd(pvdWriter, pvdRegData, *aggBatch[i]);
		}
	}
	OMNI_PVD_WRITE_SCOPE_END

	// (4) Per scene: the scene object and its membership edges (which actors / articulations /
	//     aggregates belong to it), plus the direct-GPU-API retained body / link forces that live in
	//     device state. The objects were already created in (3); from-start recording likewise emits
	//     these memberships at add time, separately from object creation.
	const physx::PxU32 nbScenes = physics.getNbScenes();
	for (physx::PxU32 si = 0; si < nbScenes; ++si)
	{
		physx::PxScene* pxScene = NULL;
		if (physics.getScenes(&pxScene, 1, si) != 1 || pxScene == NULL)
			continue;
		physx::NpScene* npScene = static_cast<physx::NpScene*>(pxScene);

		// The PxScene object itself + its membership references, opening a clean
		// pre-sim frame 1 on the new stream.
		npScene->getSceneOvdClientInternal().resetFrameId();
		npScene->emitSceneStateToOmniPvd();
		npScene->emitSceneObjectListsToOmniPvd();
		npScene->streamRigidDynamicGPUForcesToOmniPvd(); // GPU direct-API retained forces (device-side state)
		npScene->streamArticulationGPUForcesToOmniPvd(); // GPU direct-API articulation link forces (device-side state)
	}

	// If any write failed while emitting the snapshot (e.g. the late-attach reader went
	// away mid-snapshot, or a send timed out), report failure and clear the sampling flag
	// (set isSampling() back to false) so the caller can rebind a fresh stream and start again.
	const bool snapshotOk = samplerInternals->mPvdStream.dataWasWrittenSuccessfully();
	if (!snapshotOk)
	{
		physx::PxMutex::ScopedLock myLock(samplerInternals->mSampleMutex);
		samplerInternals->mIsSampling = false;
	}
	return snapshotOk;
}

#undef OVD_SNAPSHOT_PHYSICS_LIST

void OmniPvdPxSampler::setOmniPvdInstance(physx::NpOmniPvd* omniPvdInstance)
{
	samplerInternals->mPvdStream.setOmniPvdInstance(omniPvdInstance);
}

void createGeometry(OmniPvdWriter* pvdWriter, const OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxGeometry & pxGeom)
{

	switch (pxGeom.getType())
	{
	case physx::PxGeometryType::eSPHERE:
	{
		streamSphereGeometry(pvdWriter, pvdRegData, (const physx::PxSphereGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eCAPSULE:
	{
		streamCapsuleGeometry(pvdWriter, pvdRegData, (const physx::PxCapsuleGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eBOX:
	{
		streamBoxGeometry(pvdWriter, pvdRegData, (const physx::PxBoxGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eTRIANGLEMESH:
	{
		streamTriMeshGeometry(pvdWriter, pvdRegData, (const physx::PxTriangleMeshGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eTETRAHEDRONMESH:
	{
		const physx::PxTetrahedronMeshGeometry& g = static_cast<const physx::PxTetrahedronMeshGeometry&>(pxGeom);
		OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMeshGeometry, g);
		if (g.tetrahedronMesh)
		{
			streamTetMesh(pvdWriter, pvdRegData, *g.tetrahedronMesh);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMeshGeometry, tetrahedronMesh, g, g.tetrahedronMesh);
		}
	}
	break;
	case physx::PxGeometryType::eCONVEXCORE:
	{
		streamConvexCoreGeometry(pvdWriter, pvdRegData, (const physx::PxConvexCoreGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eCONVEXMESH:
	{
		streamConvexMeshGeometry(pvdWriter, pvdRegData, (const physx::PxConvexMeshGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eHEIGHTFIELD:
	{
		streamHeightFieldGeometry(pvdWriter, pvdRegData, (const physx::PxHeightFieldGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::ePLANE:
	{
		streamPlaneGeometry(pvdWriter, pvdRegData, (const physx::PxPlaneGeometry &)pxGeom);
	}
	break;
	case physx::PxGeometryType::eCUSTOM:
	{
		streamCustomGeometry(pvdWriter, pvdRegData, (const physx::PxCustomGeometry &)pxGeom);
	}
	break;
	default:
	break;
	}
}

void destroyGeometry(const physx::PxGeometry& pxGeom)
{

	switch (pxGeom.getType())
	{
	case physx::PxGeometryType::eSPHERE:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxSphereGeometry, static_cast<const PxSphereGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eCAPSULE:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxCapsuleGeometry, static_cast<const PxCapsuleGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eBOX:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxBoxGeometry, static_cast<const PxBoxGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eTRIANGLEMESH:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTriangleMeshGeometry, static_cast<const PxTriangleMeshGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eTETRAHEDRONMESH:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMeshGeometry, static_cast<const PxTetrahedronMeshGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eCONVEXMESH:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxConvexMeshGeometry, static_cast<const PxConvexMeshGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eHEIGHTFIELD:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxHeightFieldGeometry, static_cast<const PxHeightFieldGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::ePLANE:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxPlaneGeometry, static_cast<const PxPlaneGeometry&>(pxGeom));
	}
	break;
	case physx::PxGeometryType::eCUSTOM:
	{
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometry, static_cast<const PxCustomGeometry&>(pxGeom));
	}
	break;
	default:
	break;
	}
}

void streamShapeUpdateGeometry(const physx::PxShape& shape)
{
	destroyGeometry(shape.getGeometry());
	{
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		createGeometry(pvdWriter, pvdRegData, shape.getGeometry());
		OMNI_PVD_WRITE_SCOPE_END
	}
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxShape, geom, shape, &shape.getGeometry());
}

void OmniPvdPxSampler::onObjectAdd(const physx::PxBase& object)
{
	if (!isSampling()) return;

	// A single live add opens exactly one write scope (one lock acquisition) and forwards to the
	// batch variant below, which does the actual emission with the bound writer + registration data.
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	onObjectAdd(pvdWriter, pvdRegData, object);
	OMNI_PVD_WRITE_SCOPE_END
}

void OmniPvdPxSampler::onObjectAdd(OmniPvdWriter* pvdWriter, const physx::OmniPvdPxCoreRegistrationData* pvdRegData, const physx::PxBase& object)
{
	const PxPhysics& physics = static_cast<PxPhysics&>(NpPhysics::getInstance());

	switch (object.getConcreteType())
	{
		case physx::PxConcreteType::eHEIGHTFIELD:
		{
			const PxHeightField& hf = static_cast<const PxHeightField&>(object);
			streamHeightField(pvdWriter, pvdRegData, hf);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, heightFields, physics, hf);
		}
		break;
		case physx::PxConcreteType::eCONVEX_MESH:
		{
			const PxConvexMesh& cm = static_cast<const PxConvexMesh&>(object);
			streamConvexMesh(pvdWriter, pvdRegData, cm);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, convexMeshes, physics, cm);
		}
		break;
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH33:
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH34:
		{
			const PxTriangleMesh& m = static_cast<const PxTriangleMesh&>(object);
			streamTriMesh(pvdWriter, pvdRegData, m);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, triangleMeshes, physics, m);
		}
		break;
		case physx::PxConcreteType::eTETRAHEDRON_MESH:
		{
			const PxTetrahedronMesh& tm = static_cast<const PxTetrahedronMesh&>(object);
			streamTetMesh(pvdWriter, pvdRegData, tm);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tetrahedronMeshes, physics, tm);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_VOLUME_MESH:
		{
			const PxDeformableVolumeMesh& dm = static_cast<const PxDeformableVolumeMesh&>(object);
			streamDeVoMesh(pvdWriter, pvdRegData, dm);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumeMeshes, physics, dm);
		}
		break;
		case physx::PxConcreteType::eBVH:
		{
			const PxBVH& bvh = static_cast<const PxBVH&>(object);
			streamBVH(pvdWriter, pvdRegData, bvh);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, bvhs, physics, bvh);
		}
		break;
		case physx::PxConcreteType::eSHAPE:
		{
			const PxShape& shape = static_cast<const physx::PxShape&>(object);
			createGeometry(pvdWriter, pvdRegData, shape.getGeometry());
			streamShape(pvdWriter, pvdRegData, shape);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, shapes, physics, shape);
		}
		break;
		case physx::PxConcreteType::eMATERIAL:
		{
			const PxMaterial& mat = static_cast<const PxMaterial&>(object);
			streamMaterial(pvdWriter, pvdRegData, mat);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, materials, physics, mat);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_SURFACE_MATERIAL:
		{
			const PxDeformableSurfaceMaterial& dsMat = static_cast<const PxDeformableSurfaceMaterial&>(object);
			streamFEMClothMaterial(pvdWriter, pvdRegData, dsMat);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableSurfaceMaterials, physics, dsMat);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_VOLUME_MATERIAL:
		{
			const PxDeformableVolumeMaterial& sbMat = static_cast<const PxDeformableVolumeMaterial&>(object);
			streamFEMSoBoMaterial(pvdWriter, pvdRegData, sbMat);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumeMaterials, physics, sbMat);
		}
		break;
		case physx::PxConcreteType::ePBD_MATERIAL:
		{
			const PxPBDMaterial& pbdhMat = static_cast<const PxPBDMaterial&>(object);
			streamPBDMaterial(pvdWriter, pvdRegData, pbdhMat);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, PBDMaterials, physics, pbdhMat);
		}
		break;
		case physx::PxConcreteType::eAGGREGATE:
		{
			const PxAggregate& agg = static_cast<const PxAggregate&>(object);
			streamAggregate(pvdWriter, pvdRegData, agg);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, aggregates, physics, agg);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
		{
			const PxArticulationReducedCoordinate& art = static_cast<const PxArticulationReducedCoordinate&>(object);
			streamArticulation(pvdWriter, pvdRegData, art);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, articulations, physics, art);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_LINK:
		{
			const PxArticulationLink& artLink = static_cast<const PxArticulationLink&>(object);
			streamArticulationLink(pvdWriter, pvdRegData, artLink);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, links, artLink.getArticulation(), artLink);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
		{
			const PxArticulationJointReducedCoordinate& artJoint = static_cast<const PxArticulationJointReducedCoordinate&>(object);
			streamArticulationJoint(pvdWriter, pvdRegData, artJoint);
			break;
		}
		case physx::PxConcreteType::eARTICULATION_MIMIC_JOINT:
		{
			// this is added in NpScene::addArticulationMimicJointInternal
			break;
		}

		case physx::PxConcreteType::eRIGID_DYNAMIC:
		{
			const PxRigidDynamic& rd = static_cast<const PxRigidDynamic&>(object);
			streamRigidDynamic(pvdWriter, pvdRegData, rd);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidDynamics, physics, rd);
		}
		break;
		case physx::PxConcreteType::eRIGID_STATIC:
		{
			const PxRigidStatic& rs = static_cast<const PxRigidStatic&>(object);
			streamRigidStatic(pvdWriter, pvdRegData, rs);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidStatics, physics, rs);
		}
		break;

	#if PX_SUPPORT_GPU_PHYSX

		case physx::PxConcreteType::eDEFORMABLE_VOLUME:
		{
			const PxDeformableVolume& dv = static_cast<const PxDeformableVolume&>(object);
			streamDeformableVolume(pvdWriter, pvdRegData, dv);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumes, physics, dv);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_SURFACE:
		{
			const PxDeformableSurface& ds = static_cast<const PxDeformableSurface&>(object);
			streamDeformableSurface(pvdWriter, pvdRegData, ds);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableSurfaces, physics, ds);
		}
		break;

		case physx::PxConcreteType::ePBD_PARTICLESYSTEM:
		{
			const PxPBDParticleSystem& ps = static_cast<const PxPBDParticleSystem&>(object);
			streamPBDParticleSystem(pvdWriter, pvdRegData, ps);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, pbdParticleSystems, physics, ps);
		}
		break;
		case physx::PxConcreteType::ePARTICLE_BUFFER:
		{
			const PxParticleBuffer& pb = static_cast<const PxParticleBuffer&>(object);
			streamParticleBuffer(pvdWriter, pvdRegData, pb);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, particleBuffers, physics, pb);
		}
		break;
		case physx::PxConcreteType::ePARTICLE_DIFFUSE_BUFFER:
		{
			const PxParticleAndDiffuseBuffer& pb = static_cast<const PxParticleAndDiffuseBuffer&>(object);
			streamParticleAndDiffuseBuffer(pvdWriter, pvdRegData, pb);
			OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxPhysics, particleBuffers, physics, pb);
		}
		break;

#endif
	}
}

void OmniPvdPxSampler::onObjectRemove(const physx::PxBase& object)
{
	if (!isSampling()) return;

	const PxPhysics& physics = static_cast<PxPhysics&>(NpPhysics::getInstance());

	switch (object.getConcreteType())
	{
		case physx::PxConcreteType::eHEIGHTFIELD:
		{
			const PxHeightField& hf = static_cast<const PxHeightField&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, heightFields, physics, hf);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxHeightField, hf);
		}
		break;
		case physx::PxConcreteType::eCONVEX_MESH:
		{
			const PxConvexMesh& cm = static_cast<const PxConvexMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, convexMeshes, physics, cm);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxConvexMesh, cm);
		}
		break;
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH33:
		case physx::PxConcreteType::eTRIANGLE_MESH_BVH34:
		{
			const PxTriangleMesh& m = static_cast<const PxTriangleMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, triangleMeshes, physics, m);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTriangleMesh, m);
		}
		break;
		case physx::PxConcreteType::eTETRAHEDRON_MESH:
		{
			const PxTetrahedronMesh& tm = static_cast<const PxTetrahedronMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, tetrahedronMeshes, physics, tm);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxTetrahedronMesh, tm);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_VOLUME_MESH:
		{
			const PxDeformableVolumeMesh& dm = static_cast<const PxDeformableVolumeMesh&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumeMeshes, physics, dm);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMesh, dm);
		}
		break;
		case physx::PxConcreteType::eBVH:
		{
			const PxBVH& bvh = static_cast<const PxBVH&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, bvhs, physics, bvh);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxBVH, bvh);
		}
		break;
		case physx::PxConcreteType::eSHAPE:
		{
			const PxShape& shape = static_cast<const physx::PxShape&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, shapes, physics, shape);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxShape, shape);
			destroyGeometry(shape.getGeometry());
		}
		break;
		case physx::PxConcreteType::eMATERIAL:
		{
			const PxMaterial& mat = static_cast<const PxMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, materials, physics, mat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxMaterial, mat);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_SURFACE_MATERIAL:
		{
			const PxDeformableSurfaceMaterial& dsMat = static_cast<const PxDeformableSurfaceMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableSurfaceMaterials, physics, dsMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxDeformableSurfaceMaterial, dsMat);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_VOLUME_MATERIAL:
		{
			const PxDeformableVolumeMaterial& sbMat = static_cast<const PxDeformableVolumeMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumeMaterials, physics, sbMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxDeformableVolumeMaterial, sbMat);
		}
		break;
		case physx::PxConcreteType::ePBD_MATERIAL:
		{
			const PxPBDMaterial& pbdhMat = static_cast<const PxPBDMaterial&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, PBDMaterials, physics, pbdhMat);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxPBDMaterial, pbdhMat);
		}
		break;
		case physx::PxConcreteType::eAGGREGATE:
		{
			const PxAggregate& agg = static_cast<const PxAggregate&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, aggregates, physics, agg);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxAggregate, agg);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
		{
			const PxArticulationReducedCoordinate& art = static_cast<const PxArticulationReducedCoordinate&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, articulations, physics, art);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, art);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_LINK:
		{
			const PxArticulationLink& artLink = static_cast<const PxArticulationLink&>(object);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, artLink);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
		{
			const PxArticulationJointReducedCoordinate& artJoint = static_cast<const PxArticulationJointReducedCoordinate&>(object);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, artJoint);
		}
		break;
		case physx::PxConcreteType::eARTICULATION_MIMIC_JOINT:
		{
			const PxArticulationMimicJoint& artMimicJoint = static_cast<const PxArticulationMimicJoint&>(object);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxArticulationMimicJoint, artMimicJoint);
		}
		break;

		case physx::PxConcreteType::eRIGID_DYNAMIC:
		{
			const PxRigidDynamic& rd = static_cast<const PxRigidDynamic&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidDynamics, physics, rd);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, rd);
		}
		break;
		case physx::PxConcreteType::eRIGID_STATIC:
		{
			const PxRigidStatic& rs = static_cast<const PxRigidStatic&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, rigidStatics, physics, rs);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, rs);
		}
		break;
		case physx::PxConcreteType::ePBD_PARTICLESYSTEM:
		{
			const PxPBDParticleSystem& ps = static_cast<const PxPBDParticleSystem&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, pbdParticleSystems, physics, ps);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, ps);
		}
		break;
#if PX_SUPPORT_GPU_PHYSX

		case physx::PxConcreteType::eDEFORMABLE_VOLUME:
		{
			const PxDeformableVolume& dv = static_cast<const PxDeformableVolume&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableVolumes, physics, dv);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, dv);
		}
		break;
		case physx::PxConcreteType::eDEFORMABLE_SURFACE:
		{
			const PxDeformableSurface& ds = static_cast<const PxDeformableSurface&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, deformableSurfaces, physics, ds);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxActor, ds);
		}
		break;

		case physx::PxConcreteType::ePARTICLE_BUFFER:
		{
			const PxParticleBuffer& pb = static_cast<const PxParticleBuffer&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, particleBuffers, physics, pb);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, pb);
		}
		break;
		case physx::PxConcreteType::ePARTICLE_DIFFUSE_BUFFER:
		{
			//need to remove PxDiffuseParticleParams before releasing the low level object
			const PxParticleAndDiffuseBuffer& pb = static_cast<const PxParticleAndDiffuseBuffer&>(object);
			OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxPhysics, particleBuffers, physics, pb);
			OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxParticleAndDiffuseBuffer, pb);
		}
		break;

#endif
	}
}

// Returns true if the Geom was not yet seen and added
bool OmniPvdSamplerInternals::addSharedMeshIfNotSeen(const void* geom, OmniPvdSharedMeshEnum geomEnum)
{
	physx::PxMutex::ScopedLock myLock(samplerInternals->mSharedGeomsMutex);
	const physx::PxHashMap<const void*, OmniPvdSharedMeshEnum>::Entry* entry = samplerInternals->mSharedMeshesMap.find(geom);
	if (entry)
	{
		return false;
	}
	else
	{
		samplerInternals->mSharedMeshesMap[geom] = geomEnum;
		return true;
	}
}

void OmniPvdPxSampler::reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(writer, registrationData)

	// The write scope above yields a writer only while sampling (its NpOmniPvdSampling() gate), so
	// nothing below runs unless a recording is active; the schema is then registered on the bound
	// stream and the pvdPxErrorCode class handle is valid. The handle is generated by the
	// OMNI_PVD_ENUM_BEGIN(PxErrorCode) macro in OmniPvdTypes.h; to add new message types, add a new
	// enum there so the type (code) parameter can be indexed in the class data (see OmniPvdOvdParser.cpp).
	const OmniPvdClassHandle handle = registrationData->pvdPxErrorCode.classHandle;

	writer->recordMessage(OMNI_PVD_CONTEXT_HANDLE, message, file, line, code, handle);
	OMNI_PVD_WRITE_SCOPE_END
}

///////////////////////////////////////////////////////////////////////////////

OmniPvdPxSampler* OmniPvdPxSampler::getInstance()
{
	PX_ASSERT(&physx::NpPhysics::getInstance() != NULL);
	return &physx::NpPhysics::getInstance() ? physx::NpPhysics::getInstance().mOmniPvdSampler : NULL;
}

OmniPvdPxSampler* OmniPvdPxSampler::getSamplingInstance()
{
	OmniPvdPxSampler* sampler = getInstance();
	if (sampler)
	{
		return sampler->isSampling() ? sampler : NULL;
	}
	return NULL;
}


namespace physx
{

const OmniPvdPxCoreRegistrationData* NpOmniPvdGetPxCoreRegistrationData()
{
	if (samplerInternals)
	{
		return &samplerInternals->mPvdStream.mRegistrationData;
	}
	else
	{
		return NULL;
	}
}

physx::NpOmniPvd* NpOmniPvdGetInstance()
{
	if (samplerInternals)
	{
		return samplerInternals->mPvdStream.mOmniPvdInstance;
	}
	else
	{
		return NULL;
	}
}

bool NpOmniPvdSampling()
{
	OmniPvdPxSampler* sampler = OmniPvdPxSampler::getInstance();
	return sampler ? sampler->isSampling() : false;
}

}

#endif
