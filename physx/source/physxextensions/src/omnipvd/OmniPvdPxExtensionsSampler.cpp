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
