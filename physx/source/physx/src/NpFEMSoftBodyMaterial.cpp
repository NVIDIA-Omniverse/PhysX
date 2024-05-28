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

#include "NpFEMSoftBodyMaterial.h"
#include "NpPhysics.h"
#include "CmUtils.h"

#if PX_SUPPORT_GPU_PHYSX
using namespace physx;
using namespace Cm;

NpFEMSoftBodyMaterial::NpFEMSoftBodyMaterial(const PxsFEMSoftBodyMaterialCore& desc) :
	PxFEMSoftBodyMaterial(PxConcreteType::eSOFTBODY_MATERIAL, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMaterial(desc)
{
	mMaterial.mMaterial = this;  // back-reference	
}

NpFEMSoftBodyMaterial::~NpFEMSoftBodyMaterial()
{
	NpPhysics::getInstance().removeMaterialFromTable(*this);
}

// PX_SERIALIZATION
void NpFEMSoftBodyMaterial::resolveReferences(PxDeserializationContext&)
{
	// ### this one could be automated if NpMaterial would inherit from MaterialCore
	// ### well actually in that case the pointer would not even be needed....
	mMaterial.mMaterial = this;	// Resolve MaterialCore::mMaterial

	// Maybe not the best place to do it but it has to be done before the shapes resolve material indices
	// since the material index translation table is needed there. This requires that the materials have
	// been added to the table already.
	NpPhysics::getInstance().addMaterial(this);
}

void NpFEMSoftBodyMaterial::onRefCountZero()
{
	void* ud = userData;

	if (getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		NpFactory::getInstance().releaseFEMMaterialToPool(*this);
	}
	else
		this->~NpFEMSoftBodyMaterial();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(this, ud);
}

NpFEMSoftBodyMaterial* NpFEMSoftBodyMaterial::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpFEMSoftBodyMaterial* obj = PX_PLACEMENT_NEW(address, NpFEMSoftBodyMaterial(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpFEMSoftBodyMaterial);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpFEMSoftBodyMaterial::release()
{
	RefCountable_decRefCount(*this);
}

void NpFEMSoftBodyMaterial::acquireReference()
{
	RefCountable_incRefCount(*this);
}

PxU32 NpFEMSoftBodyMaterial::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

PX_INLINE void NpFEMSoftBodyMaterial::updateMaterial()
{
	NpPhysics::getInstance().updateMaterial(*this);
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::setYoungsModulus(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "NpFEMSoftBodyMaterial::setYoungsModulus: invalid float");
	
	mMaterial.youngs = x;

	updateMaterial();
}

PxReal NpFEMSoftBodyMaterial::getYoungsModulus() const
{
	return mMaterial.youngs;
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::setPoissons(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f && x <= 0.5f, "PxMaterial::setPoissons: invalid float");
	mMaterial.poissons = x;

	updateMaterial();
}

PxReal NpFEMSoftBodyMaterial::getPoissons() const
{
	return mMaterial.poissons;
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::setDynamicFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMaterial::setDynamicFriction: invalid float");
	mMaterial.dynamicFriction = x;

	updateMaterial();
}

PxReal NpFEMSoftBodyMaterial::getDynamicFriction() const
{
	return mMaterial.dynamicFriction;
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::setDamping(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f, "PxMaterial::setDamping: invalid float");
	mMaterial.damping = x;

	updateMaterial();
}

PxReal NpFEMSoftBodyMaterial::getDamping() const
{
	return mMaterial.damping;
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::setDampingScale(PxReal x)
{
	PX_CHECK_AND_RETURN(x >= 0.f && x<= 1.f, "PxMaterial::setDampingScale: invalid float, must be in [0.0, 1.0] range.");
	mMaterial.dampingScale = toUniformU16(x);

	updateMaterial();
}

PxReal NpFEMSoftBodyMaterial::getDampingScale() const
{
	return toUniformReal(mMaterial.dampingScale);
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::setMaterialModel(PxFEMSoftBodyMaterialModel::Enum model)
{
	mMaterial.materialModel = PxU16(model);

	updateMaterial();
}

PxFEMSoftBodyMaterialModel::Enum NpFEMSoftBodyMaterial::getMaterialModel() const
{
	return PxFEMSoftBodyMaterialModel::Enum(mMaterial.materialModel);
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::setDeformThreshold(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxFEMMaterial::setDeformThreshold: invalid float");

	mMaterial.deformThreshold = x;

	updateMaterial();
}

PxReal NpFEMSoftBodyMaterial::getDeformThreshold() const
{
	return mMaterial.deformThreshold;
}

/////////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::setDeformLowLimitRatio(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxFEMMaterial::setDeformLowLimitRatio: invalid float");

	mMaterial.deformLowLimitRatio = x;

	updateMaterial();
}

PxReal NpFEMSoftBodyMaterial::getDeformLowLimitRatio() const
{
	return mMaterial.deformLowLimitRatio;
}

/////////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::setDeformHighLimitRatio(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxFEMMaterial::setDeformLowLimitRatio: invalid float");

	mMaterial.deformHighLimitRatio = x;

	updateMaterial();
}

PxReal NpFEMSoftBodyMaterial::getDeformHighLimitRatio() const
{
	return mMaterial.deformHighLimitRatio;
}

///////////////////////////////////////////////////////////////////////////////
#endif

