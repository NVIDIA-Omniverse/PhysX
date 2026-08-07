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

#ifndef OMNI_PVD_EXTENSION_SAMPLER_H
#define OMNI_PVD_EXTENSION_SAMPLER_H


#if PX_SUPPORT_OMNI_PVD

#include "foundation/PxUserAllocated.h"
#include "omnipvd/PxOmniPvd.h"

#include "ExtOmniPvdRegistrationData.h"

namespace physx
{
	class PxOmniPvd;
	class PxPhysics;
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
	virtual void onStartSampling(physx::PxOmniPvd& omniPvd) PX_OVERRIDE;

	// Re-emit every joint the SDK tracks onto the currently bound write stream, for a full-state
	// snapshot. Enumerates the factory's constraints (scene-less included), filters to joints, and
	// emits each by concrete joint type. onStartSampling() re-registers the schema before calling this.
	void snapshotConstraints();

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
	physx::PxOmniPvd* mOmniPvdInstance;
	physx::PxPhysics& mPhysics;
	physx::Ext::OmniPvdPxExtensionsRegistrationData mRegistrationData;
};


namespace physx
{
namespace Ext
{

const OmniPvdPxExtensionsRegistrationData* OmniPvdGetPxExtensionsRegistrationData();
PxOmniPvd* OmniPvdGetInstance();
}
}

#endif

#endif
