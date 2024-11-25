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

#include "NpDeformableSurfaceMaterial.h"
#include "NpPhysics.h"
#include "CmUtils.h"

#if PX_SUPPORT_GPU_PHYSX
using namespace physx;
using namespace Cm;

NpDeformableSurfaceMaterial::NpDeformableSurfaceMaterial(const PxsDeformableSurfaceMaterialCore& desc) :
	PxDeformableSurfaceMaterial(PxConcreteType::eDEFORMABLE_SURFACE_MATERIAL, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMaterial(desc)
{
	mMaterial.mMaterial = this;  // back-reference	
}

NpDeformableSurfaceMaterial::~NpDeformableSurfaceMaterial()
{
	NpPhysics::getInstance().removeMaterialFromTable(*this);
}

// PX_SERIALIZATION
void NpDeformableSurfaceMaterial::resolveReferences(PxDeserializationContext&)
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

void NpDeformableSurfaceMaterial::onRefCountZero()
{
	void* ud = userData;

	if (getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		NpFactory::getInstance().releaseDeformableSurfaceMaterialToPool(*this);
	}
	else
		this->~NpDeformableSurfaceMaterial();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(this, ud);
}

NpDeformableSurfaceMaterial* NpDeformableSurfaceMaterial::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpDeformableSurfaceMaterial* obj = PX_PLACEMENT_NEW(address, NpDeformableSurfaceMaterial(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpDeformableSurfaceMaterial);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpDeformableSurfaceMaterial::release()
{
	RefCountable_decRefCount(*this);
}

void NpDeformableSurfaceMaterial::acquireReference()
{
	RefCountable_incRefCount(*this);
}

PxU32 NpDeformableSurfaceMaterial::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

PX_INLINE void NpDeformableSurfaceMaterial::updateMaterial()
{
	NpPhysics::getInstance().updateMaterial(*this);
}

///////////////////////////////////////////////////////////////////////////////

void NpDeformableSurfaceMaterial::setYoungsModulus(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxDeformableSurfaceMaterial::setYoungsModulus: invalid float");

	mMaterial.youngs = x;

	updateMaterial();
}

PxReal NpDeformableSurfaceMaterial::getYoungsModulus() const
{
	return mMaterial.youngs;
}

///////////////////////////////////////////////////////////////////////////////

void NpDeformableSurfaceMaterial::setPoissons(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f && x < 0.5f, "PxDeformableSurfaceMaterial::setPoissons: invalid float");
	mMaterial.poissons = x;

	updateMaterial();
}

PxReal NpDeformableSurfaceMaterial::getPoissons() const
{
	return mMaterial.poissons;
}

///////////////////////////////////////////////////////////////////////////////

void NpDeformableSurfaceMaterial::setDynamicFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxDeformableSurfaceMaterial::setDynamicFriction: invalid float");
	mMaterial.dynamicFriction = x;

	updateMaterial();
}

PxReal NpDeformableSurfaceMaterial::getDynamicFriction() const
{
	return mMaterial.dynamicFriction;
}

///////////////////////////////////////////////////////////////////////////////

void NpDeformableSurfaceMaterial::setThickness(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxDeformableSurfaceMaterial::setThickness: invalid float");
	mMaterial.thickness = x;

	updateMaterial();
}

PxReal NpDeformableSurfaceMaterial::getThickness() const
{
	return mMaterial.thickness;
}

void NpDeformableSurfaceMaterial::setBendingStiffness(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxDeformableSurfaceMaterial::setBendingStiffness: invalid float");
	mMaterial.bendingStiffness = x;

	updateMaterial();
}

PxReal NpDeformableSurfaceMaterial::getBendingStiffness() const
{
	return mMaterial.bendingStiffness;
}

void NpDeformableSurfaceMaterial::setElasticityDamping(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxDeformableSurfaceMaterial::setElasticityDamping: invalid float");
	mMaterial.damping = x;

	updateMaterial();
}

PxReal NpDeformableSurfaceMaterial::getElasticityDamping() const
{
	return mMaterial.damping;
}

void NpDeformableSurfaceMaterial::setBendingDamping(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxDeformableSurfaceMaterial::setBendingDamping: invalid float");
	mMaterial.bendingDamping = x;

	updateMaterial();
}

PxReal NpDeformableSurfaceMaterial::getBendingDamping() const
{
	return mMaterial.bendingDamping;
}

//////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
#endif // PX_SUPPORT_GPU_PHYSX

