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

#include "NpFEMClothMaterial.h"
#include "NpPhysics.h"
#include "CmUtils.h"

#if PX_SUPPORT_GPU_PHYSX
using namespace physx;
using namespace Cm;

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION

NpFEMClothMaterial::NpFEMClothMaterial(const PxsFEMClothMaterialCore& desc) :
	PxFEMClothMaterial(PxConcreteType::eCLOTH_MATERIAL, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMaterial(desc)
{
	mMaterial.mMaterial = this;  // back-reference	
}

NpFEMClothMaterial::~NpFEMClothMaterial()
{
	NpPhysics::getInstance().removeMaterialFromTable(*this);
}

// PX_SERIALIZATION
void NpFEMClothMaterial::resolveReferences(PxDeserializationContext&)
{
	// ### this one could be automated if NpMaterial would inherit from MaterialCore
	// ### well actually in that case the pointer would not even be needed....
	mMaterial.mMaterial = this;	// Resolve MaterialCore::mMaterial

	// Maybe not the best place to do it but it has to be done before the shapes resolve material indices
	// since the material index translation table is needed there. This requires that the materials have
	// been added to the table already.
	// PT: TODO: why commented out?
	//NpPhysics::getInstance().addFEMMaterial(this);
}

void NpFEMClothMaterial::onRefCountZero()
{
	void* ud = userData;

	if (getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		NpFactory::getInstance().releaseFEMClothMaterialToPool(*this);
	}
	else
		this->~NpFEMClothMaterial();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(this, ud);
}

NpFEMClothMaterial* NpFEMClothMaterial::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpFEMClothMaterial* obj = PX_PLACEMENT_NEW(address, NpFEMClothMaterial(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpFEMClothMaterial);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpFEMClothMaterial::release()
{
	RefCountable_decRefCount(*this);
}

void NpFEMClothMaterial::acquireReference()
{
	RefCountable_incRefCount(*this);
}

PxU32 NpFEMClothMaterial::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

PX_INLINE void NpFEMClothMaterial::updateMaterial()
{
	NpPhysics::getInstance().updateMaterial(*this);
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMClothMaterial::setYoungsModulus(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxFEMClothMaterial::setYoungsModulus: invalid float");

	mMaterial.youngs = x;

	updateMaterial();
}

PxReal NpFEMClothMaterial::getYoungsModulus() const
{
	return mMaterial.youngs;
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMClothMaterial::setPoissons(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f && x < 0.5f, "PxFEMClothMaterial::setPoissons: invalid float");
	mMaterial.poissons = x;

	updateMaterial();
}

PxReal NpFEMClothMaterial::getPoissons() const
{
	return mMaterial.poissons;
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMClothMaterial::setDynamicFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxFEMClothMaterial::setDynamicFriction: invalid float");
	mMaterial.dynamicFriction = x;

	updateMaterial();
}

PxReal NpFEMClothMaterial::getDynamicFriction() const
{
	return mMaterial.dynamicFriction;
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMClothMaterial::setThickness(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxFEMClothMaterial::setThickness: invalid float");
	mMaterial.thickness = x;

	updateMaterial();
}

PxReal NpFEMClothMaterial::getThickness() const
{
	return mMaterial.thickness;
}

//////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
#endif
#endif
