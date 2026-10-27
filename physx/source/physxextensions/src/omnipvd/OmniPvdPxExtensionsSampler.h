// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_EXTENSION_SAMPLER_H
#define OMNI_PVD_EXTENSION_SAMPLER_H


#if PX_SUPPORT_OMNI_PVD

#include "foundation/PxUserAllocated.h"

#include "ExtOmniPvdRegistrationData.h"

namespace physx
{
	class PxOmniPvd;
}


class OmniPvdPxExtensionsSampler : public physx::PxUserAllocated
{
public:
	OmniPvdPxExtensionsSampler();
	~OmniPvdPxExtensionsSampler();
	void setOmniPvdInstance(physx::PxOmniPvd* omniPvdInstance);
	physx::PxOmniPvd* getOmniPvdInstance();
	void registerClasses();

	const physx::Ext::OmniPvdPxExtensionsRegistrationData& getRegistrationData() const { return mRegistrationData; }

	// OmniPvdPxExtensionsSampler singleton
	static bool createInstance();
	static OmniPvdPxExtensionsSampler* getInstance();
	static void destroyInstance();

private:
	physx::PxOmniPvd* mOmniPvdInstance;
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
