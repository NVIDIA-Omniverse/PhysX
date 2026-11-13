// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_EXTENSION_SAMPLER_H
#define OMNI_PVD_EXTENSION_SAMPLER_H


#if PX_SUPPORT_OMNI_PVD

#include "foundation/PxUserAllocated.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
#include "omnipvd/PxOmniPvd.h"

#include "ExtOmniPvdRegistrationData.h"

namespace physx
{
	class PxOmniPvd;
	class PxPhysics;
	class PxConstraint;
	class PxShape;
}


// Implements PxOmniPvdEventCallback so the extensions module contributes its objects (joints)
// to a full-state snapshot: it is registered with the bound PxOmniPvd in PxInitExtensions
// and its onStartSampling() fires whenever PxOmniPvd::startSampling() takes a snapshot,
// re-emitting the live extension joints and custom-geometry callbacks onto the freshly-bound
// stream. PhysXCore owns the PxOmniPvdEventCallback interface and never references PhysXExtensions.
class OmniPvdPxExtensionsSampler : public physx::PxOmniPvdEventCallback, public physx::PxUserAllocated
{
public:
	// physics is the PxPhysics whose joints onStartSampling() re-emits; it is required for the
	// sampler's whole lifetime, so it is taken here rather than through a later setter.
	OmniPvdPxExtensionsSampler(physx::PxPhysics& physics);
	~OmniPvdPxExtensionsSampler();
	void setOmniPvdInstance(physx::PxOmniPvd* omniPvdInstance);
	physx::PxOmniPvd* getOmniPvdInstance();
	void registerClasses();

	// PxOmniPvdEventCallback: called by startSampling() after the core world is emitted. Re-registers
	// the schema, then re-emits the live extension joints and custom-geometry callbacks onto the bound
	// stream (registerClasses + snapshotConstraints + snapshotCustomGeometryCallbacks).
	virtual void onStartSampling(physx::PxOmniPvd& omniPvd, const physx::PxOmniPvdFilterDesc* filter) PX_OVERRIDE;
	virtual void onObjectCapture(physx::PxOmniPvd& omniPvd, const physx::PxBase& object) PX_OVERRIDE;
	virtual void onCaptureReset(physx::PxOmniPvd& omniPvd, const physx::PxOmniPvdFilterDesc& filter) PX_OVERRIDE;

	bool shouldRecordCreation(const char* className, physx::PxU64 objectHandle);
	bool shouldRecordDestruction(physx::PxU64 objectHandle);
	bool shouldRecordValue(const char* className, const char* attributeName, physx::PxU64 objectHandle) const;
	const physx::PxBase* captureJointReference(const physx::PxBase* joint, OmniPvdWriter* writer);

	// Re-emit every joint the SDK tracks onto the currently bound write stream, for a full-state
	// snapshot. Enumerates the factory's constraints (scene-less included), filters to joints, and
	// emits each by concrete joint type. onStartSampling() re-registers the schema before calling this.
	void snapshotConstraints();
	// Called under the writer lock with the unfiltered references of a recorded gear or rack.
	void cacheJointReferences(physx::PxU64 dependent, const physx::PxBase* joint0, const physx::PxBase* joint1);

	// Re-emit every live PxCustomGeometryExt callbacks object (cylinder / cone, with their base
	// margin) onto the currently bound write stream, for a full-state snapshot. These are emitted
	// live in the callback constructors, so a late-attach recording must reproduce them here. Walks
	// the SDK's shapes via public PxPhysics API, dedups shared callbacks, and emits each once.
	void snapshotCustomGeometryCallbacks();

	const physx::Ext::OmniPvdPxExtensionsRegistrationData& getRegistrationData() const { return mRegistrationData; }

	// OmniPvdPxExtensionsSampler singleton
	static bool createInstance(physx::PxPhysics& physics);
	static OmniPvdPxExtensionsSampler* getInstance();
	static void destroyInstance();

private:
	void snapshotConstraint(physx::PxConstraint& constraint, OmniPvdWriter* writer);
	void snapshotCustomGeometry(const physx::PxShape& shape, OmniPvdWriter* writer);
	physx::PxOmniPvd* mOmniPvdInstance;
	physx::PxPhysics& mPhysics;
	bool mRecordSecondaryData;
	bool mRecordAllObjects;
	physx::Ext::OmniPvdPxExtensionsRegistrationData mRegistrationData;
	physx::PxHashSet<physx::PxU64> mEmittedObjects;
	physx::PxHashSet<physx::PxU64> mRefreshingJoints;
	struct JointReferences { physx::PxU64 joint0; physx::PxU64 joint1; };
	physx::PxHashMap<physx::PxU64, JointReferences> mJointReferences;
	physx::PxHashSet<const void*> mSelectedCustomCallbacks;
};


namespace physx
{
namespace Ext
{

const OmniPvdPxExtensionsRegistrationData* OmniPvdGetPxExtensionsRegistrationData();
PxOmniPvd* OmniPvdGetInstance();
bool OmniPvdShouldRecordCreation(const char* className, PxU64 objectHandle);
bool OmniPvdShouldRecordDestruction(PxU64 objectHandle);
bool OmniPvdShouldRecordValue(const char* className, const char* attributeName, PxU64 objectHandle);
bool OmniPvdCaptureJoint(const PxJoint& joint);
const PxBase* OmniPvdCaptureJointReference(const PxBase* joint, OmniPvdWriter* writer);
void OmniPvdCacheJointReferences(const PxJoint& dependent, const PxBase* joint0, const PxBase* joint1);
}
}

#endif

#endif
