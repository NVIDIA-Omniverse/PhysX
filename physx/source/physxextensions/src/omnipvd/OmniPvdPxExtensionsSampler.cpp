// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#if PX_SUPPORT_OMNI_PVD

#include "OmniPvdPxExtensionsSampler.h"
#include "omnipvd/PxOmniPvd.h"


using namespace physx;


void OmniPvdPxExtensionsSampler::registerClasses()
{
	PxOmniPvd::ScopedExclusiveWriter scope(mOmniPvdInstance);
	OmniPvdWriter* writer = scope.getWriter();
	if (writer)
	{
		mRegistrationData.registerData(*mOmniPvdInstance->getWriter());
	}
}

OmniPvdPxExtensionsSampler::OmniPvdPxExtensionsSampler()
{
	mOmniPvdInstance = NULL;
}

OmniPvdPxExtensionsSampler::~OmniPvdPxExtensionsSampler()
{
}

void OmniPvdPxExtensionsSampler::setOmniPvdInstance(physx::PxOmniPvd* omniPvdInstance)
{
	mOmniPvdInstance = omniPvdInstance;
}

physx::PxOmniPvd* OmniPvdPxExtensionsSampler::getOmniPvdInstance() {
	return mOmniPvdInstance;
}


///////////////////////////////////////////////////////////////////////////////

static OmniPvdPxExtensionsSampler* gOmniPvdPxExtensionsSampler = NULL;

bool OmniPvdPxExtensionsSampler::createInstance()
{
	gOmniPvdPxExtensionsSampler = PX_NEW(OmniPvdPxExtensionsSampler)();
	return gOmniPvdPxExtensionsSampler != NULL;
}

OmniPvdPxExtensionsSampler* OmniPvdPxExtensionsSampler::getInstance()
{
	return gOmniPvdPxExtensionsSampler;
}

void OmniPvdPxExtensionsSampler::destroyInstance()
{
	PX_DELETE(gOmniPvdPxExtensionsSampler);
}


namespace physx
{
namespace Ext
{

const OmniPvdPxExtensionsRegistrationData* OmniPvdGetPxExtensionsRegistrationData()
{
	OmniPvdPxExtensionsSampler* sampler = OmniPvdPxExtensionsSampler::getInstance();
	if (sampler)
	{
		return &sampler->getRegistrationData();
	}
	else
	{
		return NULL;
	}
}

PxOmniPvd* OmniPvdGetInstance()
{
	OmniPvdPxExtensionsSampler* sampler = OmniPvdPxExtensionsSampler::getInstance();
	if (sampler)
	{
		return sampler->getOmniPvdInstance();
	}
	else
	{
		return NULL;
	}
}

}
}

#endif
