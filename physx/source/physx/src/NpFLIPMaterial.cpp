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

#include "NpFLIPMaterial.h"
#include "NpPhysics.h"
#include "CmUtils.h"

#if PX_SUPPORT_GPU_PHYSX
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
using namespace physx;
using namespace Cm;

NpFLIPMaterial::NpFLIPMaterial(const PxsFLIPMaterialCore& desc) :
	PxFLIPMaterial(PxConcreteType::eFLIP_MATERIAL, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMaterial(desc)
{
	mMaterial.mMaterial = this;  // back-reference	
}

NpFLIPMaterial::~NpFLIPMaterial()
{
	NpPhysics::getInstance().removeMaterialFromTable(*this);
}

// PX_SERIALIZATION
void NpFLIPMaterial::resolveReferences(PxDeserializationContext&)
{
	// ### this one could be automated if NpMaterial would inherit from MaterialCore
	// ### well actually in that case the pointer would not even be needed....
	mMaterial.mMaterial = this;	// Resolve MaterialCore::mMaterial

	// Maybe not the best place to do it but it has to be done before the shapes resolve material indices
	// since the material index translation table is needed there. This requires that the materials have
	// been added to the table already.
	// PT: TODO: missing line here?
}

void NpFLIPMaterial::onRefCountZero()
{
	void* ud = userData;

	if (getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		NpFactory::getInstance().releaseFLIPMaterialToPool(*this);
	}
	else
		this->~NpFLIPMaterial();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(this, ud);
}

NpFLIPMaterial* NpFLIPMaterial::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpFLIPMaterial* obj = PX_PLACEMENT_NEW(address, NpFLIPMaterial(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpFLIPMaterial);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpFLIPMaterial::release()
{
	RefCountable_decRefCount(*this);
}

void NpFLIPMaterial::acquireReference()
{
	RefCountable_incRefCount(*this);
}

PxU32 NpFLIPMaterial::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

PX_INLINE void NpFLIPMaterial::updateMaterial()
{
	NpPhysics::getInstance().updateMaterial(*this);
}

///////////////////////////////////////////////////////////////////////////////

void NpFLIPMaterial::setFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxFLIPMaterial::setFriction: invalid float");

	mMaterial.friction = x;

	updateMaterial();
}

PxReal NpFLIPMaterial::getFriction() const
{
	return mMaterial.friction;
}

///////////////////////////////////////////////////////////////////////////////

void NpFLIPMaterial::setViscosity(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxFLIPMaterial::setViscosity: invalid float");
	mMaterial.viscosity = x;

	updateMaterial();
}

PxReal NpFLIPMaterial::getViscosity() const
{
	return mMaterial.viscosity;
}

///////////////////////////////////////////////////////////////////////////////

void NpFLIPMaterial::setDamping(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxFLIPMaterial::setDamping: invalid float");
	mMaterial.damping = x;

	updateMaterial();
}

PxReal NpFLIPMaterial::getDamping() const
{
	return mMaterial.damping;
}

///////////////////////////////////////////////////////////////////////////////

void NpFLIPMaterial::setAdhesion(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxFLIPMaterial::setAdhesion: invalid float");
	mMaterial.adhesion = x;

	updateMaterial();
}

PxReal NpFLIPMaterial::getAdhesion() const
{
	return mMaterial.adhesion;
}

///////////////////////////////////////////////////////////////////////////////

void NpFLIPMaterial::setGravityScale(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxFLIPMaterial::setAdhesion: invalid float");
	mMaterial.gravityScale = x;

	updateMaterial();
}

PxReal NpFLIPMaterial::getGravityScale() const
{
	return mMaterial.gravityScale;
}

///////////////////////////////////////////////////////////////////////////////

void NpFLIPMaterial::setAdhesionRadiusScale(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxFLIPMaterial::setAdhesionRadiusScale: scale must be positive");
	mMaterial.adhesionRadiusScale = x;

	updateMaterial();
}
PxReal NpFLIPMaterial::getAdhesionRadiusScale() const
{
	return mMaterial.adhesionRadiusScale;
}

///////////////////////////////////////////////////////////////////////////////

#endif
#endif
